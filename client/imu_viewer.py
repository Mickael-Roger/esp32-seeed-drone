#!/usr/bin/env python3
"""ICM-20948 quaternion 3D viewer.

Subscribes to the ESP32 over HTTP, then receives quaternion frames over UDP
and renders the IMU orientation in a 3D window.

    python imu_viewer.py <esp32-ip>
"""
import argparse
import math
import socket
import struct
import sys
import threading
import time

import numpy as np
import pyqtgraph.opengl as gl
import requests
from PySide6 import QtCore, QtGui, QtWidgets


SUBSCRIBE_INTERVAL = 2.0   # ESP32 expires subscribers after 5 s
PACKET_SIZE = 16           # 4 × float32, little-endian


def quat_to_qmatrix(w, x, y, z):
    return QtGui.QMatrix4x4(
        1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y), 0,
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x), 0,
            2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0,
                              0,                       0,                       0, 1,
    )


def quat_inv(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _box_mesh(size, center=(0.0, 0.0, 0.0), yaw=0.0, color=(0.5, 0.5, 0.5, 1.0)):
    """Vertices/faces/face-colors for an axis-aligned box, optionally rotated about Z."""
    sx, sy, sz = size
    offsets = np.array([
        [-sx/2, -sy/2, -sz/2], [+sx/2, -sy/2, -sz/2],
        [+sx/2, +sy/2, -sz/2], [-sx/2, +sy/2, -sz/2],
        [-sx/2, -sy/2, +sz/2], [+sx/2, -sy/2, +sz/2],
        [+sx/2, +sy/2, +sz/2], [-sx/2, +sy/2, +sz/2],
    ], dtype=np.float32)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    verts = (offsets @ R.T) + np.array(center, dtype=np.float32)
    faces = np.array([
        [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3],
        [3, 7, 6], [3, 6, 2], [0, 1, 5], [0, 5, 4],
        [4, 5, 6], [4, 6, 7], [0, 3, 2], [0, 2, 1],
    ], dtype=np.uint32)
    colors = np.tile(np.array(color, dtype=np.float32), (12, 1))
    return verts, faces, colors


def _disc_mesh(radius, height, segments, center=(0.0, 0.0, 0.0),
               color=(0.5, 0.5, 0.5, 1.0)):
    """Short cylinder along Z (a propeller-shaped disc) with closed top and bottom."""
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False, dtype=np.float32)
    cos_a, sin_a = np.cos(angles), np.sin(angles)
    top = np.stack([radius * cos_a, radius * sin_a,
                    np.full(segments, +height / 2, dtype=np.float32)], axis=1)
    bot = np.stack([radius * cos_a, radius * sin_a,
                    np.full(segments, -height / 2, dtype=np.float32)], axis=1)
    top_c = np.array([[0, 0, +height / 2]], dtype=np.float32)
    bot_c = np.array([[0, 0, -height / 2]], dtype=np.float32)
    verts = np.vstack([top, bot, top_c, bot_c]) + np.array(center, dtype=np.float32)

    n = segments
    top_ci, bot_ci = 2 * n, 2 * n + 1
    faces = []
    for i in range(n):
        faces.append([top_ci, i, (i + 1) % n])               # top fan
    for i in range(n):
        faces.append([bot_ci, n + (i + 1) % n, n + i])       # bottom fan (flipped)
    for i in range(n):                                       # side wall
        a, b = i, (i + 1) % n
        faces.append([a, n + a, n + b])
        faces.append([a, n + b, b])
    faces = np.array(faces, dtype=np.uint32)
    colors = np.tile(np.array(color, dtype=np.float32), (len(faces), 1))
    return verts, faces, colors


def _combine_meshes(*parts):
    """Concatenate multiple (verts, faces, colors) tuples into a single mesh."""
    vs, fs, cs, off = [], [], [], 0
    for v, f, c in parts:
        vs.append(v)
        fs.append(f + off)
        cs.append(c)
        off += len(v)
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


class VideoReceiver(QtCore.QObject):
    """Pulls MJPEG from `http://<esp_ip>/stream` and emits decoded QImage frames."""

    frame = QtCore.Signal(QtGui.QImage)

    def __init__(self, esp_ip):
        super().__init__()
        self._url = f"http://{esp_ip}:81/stream"
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            try:
                with requests.get(self._url, stream=True, timeout=5) as r:
                    if not r.ok:
                        print(f"video → {r.status_code}", file=sys.stderr)
                        self._stop.wait(2)
                        continue
                    boundary = self._extract_boundary(r.headers.get("Content-Type", ""))
                    if not boundary:
                        print(f"video: no boundary in Content-Type", file=sys.stderr)
                        self._stop.wait(2)
                        continue
                    self._parse_mjpeg(r.iter_content(chunk_size=8192), boundary)
            except requests.RequestException as e:
                print(f"video error: {e}", file=sys.stderr)
                self._stop.wait(2)

    @staticmethod
    def _extract_boundary(content_type):
        for part in content_type.split(";"):
            part = part.strip()
            if part.startswith("boundary="):
                return part.split("=", 1)[1].strip()
        return None

    def _parse_mjpeg(self, chunks, boundary):
        sep = ("--" + boundary).encode()
        buf = b""
        for chunk in chunks:
            if self._stop.is_set():
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                idx = buf.find(sep)
                if idx < 0:
                    break
                hdr_end = buf.find(b"\r\n\r\n", idx)
                if hdr_end < 0:
                    break
                clen = self._parse_content_length(buf[idx:hdr_end])
                if clen is None:
                    buf = buf[hdr_end + 4:]
                    continue
                body_start = hdr_end + 4
                if len(buf) < body_start + clen:
                    break  # need more data
                jpeg_bytes = buf[body_start:body_start + clen]
                buf = buf[body_start + clen:]
                img = QtGui.QImage.fromData(jpeg_bytes)
                if not img.isNull():
                    self.frame.emit(img)

    @staticmethod
    def _parse_content_length(headers_bytes):
        try:
            text = headers_bytes.decode(errors="ignore")
        except Exception:
            return None
        for line in text.split("\r\n"):
            if line.lower().startswith("content-length:"):
                try:
                    return int(line.split(":", 1)[1].strip())
                except ValueError:
                    return None
        return None


class Receiver(QtCore.QObject):
    quat = QtCore.Signal(float, float, float, float)

    def __init__(self, esp_ip, port=0):
        super().__init__()
        self._esp_ip = esp_ip
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._port = self._sock.getsockname()[1]
        self._stop = threading.Event()

    @property
    def port(self):
        return self._port

    def start(self):
        threading.Thread(target=self._recv_loop, daemon=True).start()
        threading.Thread(target=self._subscribe_loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        self._sock.close()

    def _subscribe_loop(self):
        url = f"http://{self._esp_ip}/subscribe"
        body = {"port": self._port}
        while not self._stop.is_set():
            try:
                r = requests.post(url, json=body, timeout=2.0)
                if not r.ok:
                    print(f"subscribe → {r.status_code} {r.text}", file=sys.stderr)
            except requests.RequestException as e:
                print(f"subscribe error: {e}", file=sys.stderr)
            self._stop.wait(SUBSCRIBE_INTERVAL)

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(PACKET_SIZE)
            except OSError:
                break
            if len(data) != PACKET_SIZE:
                continue
            w, x, y, z = struct.unpack("<ffff", data)
            self.quat.emit(w, x, y, z)


class Viewer(QtWidgets.QMainWindow):

    INSET_SIZE = 280
    INSET_MARGIN = 16

    def __init__(self, esp_ip, port=0):
        super().__init__()
        self.setWindowTitle(f"Drone viewer — {esp_ip}")
        self.resize(1024, 768)

        # Central video display
        self._video = QtWidgets.QLabel("waiting for video…")
        self._video.setAlignment(QtCore.Qt.AlignCenter)
        self._video.setStyleSheet(
            "background-color: #000; color: #888; font-family: monospace;"
        )
        self.setCentralWidget(self._video)

        # 3D inset (parented to the video label so it floats over it)
        self._gl = gl.GLViewWidget(self._video)
        self._gl.setCameraPosition(distance=3.5, elevation=25, azimuth=45)
        self._gl.resize(self.INSET_SIZE, self.INSET_SIZE)
        self._gl.setStyleSheet("border: 1px solid rgba(255,255,255,80);")

        grid = gl.GLGridItem()
        grid.setSize(4, 4)
        grid.setSpacing(0.5, 0.5)
        self._gl.addItem(grid)

        ax = gl.GLAxisItem()
        ax.setSize(1.5, 1.5, 1.5)
        self._gl.addItem(ax)

        self._drone = self._make_drone()
        self._gl.addItem(self._drone)

        self._status = self.statusBar()
        self._status.showMessage("waiting for data…   (T = tare, R = reset tare)")

        self._rx = Receiver(esp_ip, port)
        self._rx.quat.connect(self._on_quat)
        self._rx.start()
        print(f"listening on UDP port {self._rx.port}, subscribing to {esp_ip}")

        self._video_rx = VideoReceiver(esp_ip)
        self._video_rx.frame.connect(self._on_video_frame)
        self._video_rx.start()
        print(f"video stream: http://{esp_ip}/stream")

        self._frames = 0
        self._last_t = time.monotonic()
        self._last_quat = (1.0, 0.0, 0.0, 0.0)
        self._tare = None  # set to a quaternion to enable tare
        self._last_video_pixmap = None
        self._video_frames = 0
        self._last_video_t = time.monotonic()
        self._video_fps = 0.0

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._reposition_inset()
        if self._last_video_pixmap is not None:
            self._refresh_video_pixmap()

    def _reposition_inset(self):
        if self._video.width() == 0 or self._video.height() == 0:
            return
        x = self._video.width()  - self._gl.width()  - self.INSET_MARGIN
        y = self._video.height() - self._gl.height() - self.INSET_MARGIN
        self._gl.move(x, y)
        self._gl.raise_()

    def _refresh_video_pixmap(self):
        if self._last_video_pixmap is None:
            return
        scaled = self._last_video_pixmap.scaled(
            self._video.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self._video.setPixmap(scaled)
        self._reposition_inset()

    @QtCore.Slot(QtGui.QImage)
    def _on_video_frame(self, img):
        self._last_video_pixmap = QtGui.QPixmap.fromImage(img)
        self._refresh_video_pixmap()
        self._video_frames += 1
        now = time.monotonic()
        if now - self._last_video_t >= 1.0:
            self._video_fps = self._video_frames / (now - self._last_video_t)
            self._video_frames = 0
            self._last_video_t = now

    @staticmethod
    def _make_drone():
        # Body frame: +X = forward, +Z = up. Front arms are red.
        BODY  = (0.32, 0.32, 0.10)
        ARM   = (0.55, 0.06, 0.04)   # length, width, thickness
        MOTOR = (0.12, 0.12, 0.08)
        PROP_R, PROP_H = 0.20, 0.012

        DARK  = (0.15, 0.15, 0.18, 1)
        GRAY  = (0.40, 0.40, 0.45, 1)
        RED   = (0.85, 0.20, 0.20, 1)
        LIGHT = (0.55, 0.55, 0.60, 1)

        parts = [_box_mesh(BODY, color=DARK)]
        arm_len = ARM[0]

        # Yaw of each arm (radians) and color (front = red, back = gray)
        arms = [
            ( math.pi / 4, RED),     # front-left  (+X+Y)
            (-math.pi / 4, RED),     # front-right (+X-Y)
            ( 3 * math.pi / 4, GRAY),  # back-left   (-X+Y)
            (-3 * math.pi / 4, GRAY),  # back-right  (-X-Y)
        ]
        for yaw, color in arms:
            cx = (arm_len / 2) * math.cos(yaw)
            cy = (arm_len / 2) * math.sin(yaw)
            parts.append(_box_mesh(ARM, center=(cx, cy, 0), yaw=yaw, color=color))
            mx = arm_len * math.cos(yaw)
            my = arm_len * math.sin(yaw)
            parts.append(_box_mesh(MOTOR, center=(mx, my, 0), color=DARK))
            parts.append(_disc_mesh(PROP_R, PROP_H, 16,
                                    center=(mx, my, MOTOR[2] / 2 + PROP_H / 2),
                                    color=LIGHT))

        verts, faces, colors = _combine_meshes(*parts)
        md = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
        return gl.GLMeshItem(meshdata=md, smooth=False, drawEdges=True,
                             edgeColor=(0, 0, 0, 0.4))

    @QtCore.Slot(float, float, float, float)
    def _on_quat(self, w, x, y, z):
        self._last_quat = (w, x, y, z)
        if self._tare is not None:
            dw, dx, dy, dz = quat_mul(quat_inv(self._tare), (w, x, y, z))
        else:
            dw, dx, dy, dz = w, x, y, z
        self._drone.setTransform(quat_to_qmatrix(dw, dx, dy, dz))

        self._frames += 1
        now = time.monotonic()
        if now - self._last_t >= 1.0:
            fps = self._frames / (now - self._last_t)
            tare = "tared" if self._tare is not None else "raw"
            self._status.showMessage(
                f"orientation {fps:.1f} Hz [{tare}]   "
                f"video {self._video_fps:.1f} fps   "
                f"q=[w={dw:+.4f} x={dx:+.4f} y={dy:+.4f} z={dz:+.4f}]   "
                f"(T = tare, R = reset)"
            )
            self._frames = 0
            self._last_t = now

    def keyPressEvent(self, ev):
        if ev.key() == QtCore.Qt.Key_T:
            self._tare = self._last_quat
        elif ev.key() == QtCore.Qt.Key_R:
            self._tare = None
        else:
            super().keyPressEvent(ev)

    def closeEvent(self, ev):
        self._rx.stop()
        self._video_rx.stop()
        super().closeEvent(ev)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("esp_ip", help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=9000,
                        help="local UDP port to bind (0 = ephemeral, default 9000)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = Viewer(args.esp_ip, args.port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
