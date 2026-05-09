#!/usr/bin/env python3
"""ICM-20948 quaternion viewer with live drone camera and keyboard control.

The ESP32 runs as a WiFi access point. This script:
1. Reads SSID/PSK/IPs/ports from ../main/wifi_credentials.h
2. Connects the given WiFi interface to the ESP's AP via nmcli
3. Listens for orientation UDP, fetches MJPEG video, sends control packets

    python imu_viewer.py <wifi-device>
"""
import argparse
import math
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import pyqtgraph.opengl as gl
from PySide6 import QtCore, QtGui, QtWidgets


PACKET_SIZE = 16           # 4 × float32, little-endian
FLIGHT_INTERVAL = 0.05     # 50 ms — 20 Hz

VIDEO_HEADER_SIZE = 8      # u32 frame_id LE + u16 idx LE + u16 total LE
VIDEO_RECV_BUF    = 2048   # max ESP fragment is ~1408 B

FC_HEADER     = 0x66
FC_TAIL       = 0x99
FC_PREFIX_CMD = 0x03
FC_NEUTRAL    = 128
FC_FULL_HIGH  = 255
FC_FULL_LOW   = 1


def parse_credentials(path):
    """Pull #define KEY VALUE lines out of wifi_credentials.h."""
    with open(path) as f:
        text = f.read()
    cfg = {}
    for m in re.finditer(r'#define\s+(\w+)\s+"([^"]+)"', text):
        cfg[m.group(1)] = m.group(2)
    for m in re.finditer(r'#define\s+(\w+)\s+(-?\d+)\s*$', text, flags=re.MULTILINE):
        if m.group(1) not in cfg:
            cfg[m.group(1)] = int(m.group(2))
    required = ["WIFI_SSID", "WIFI_PASSWORD", "ESP_IP", "CLIENT_IP",
                "ORIENTATION_PORT", "FLIGHT_PORT", "VIDEO_PORT"]
    missing = [k for k in required if k not in cfg]
    if missing:
        sys.exit(f"missing keys in {path}: {', '.join(missing)}")
    return cfg


def _nmcli(args, check=True):
    r = subprocess.run(["nmcli"] + args, capture_output=True, text=True)
    if check and r.returncode != 0:
        sys.exit(f"nmcli {' '.join(args)} → {r.stderr.strip() or r.stdout.strip()}")
    return r


def _existing_connections():
    r = _nmcli(["-t", "-f", "NAME", "connection", "show"], check=False)
    return {line.strip() for line in r.stdout.splitlines() if line.strip()}


def _visible_ssids(device):
    r = _nmcli(["-t", "-f", "SSID", "device", "wifi", "list", "ifname", device],
               check=False)
    return {line.strip() for line in r.stdout.splitlines() if line.strip()}


def connect_to_ap(device, ssid, password):
    """Bring `device` up on the drone AP via NetworkManager.

    Creates (or refreshes) a dedicated WPA-PSK profile, triggers a scan, waits
    for the SSID to show up, then activates the profile. Idempotent.
    """
    conn_name = f"esp32-drone-{ssid}"

    if conn_name in _existing_connections():
        _nmcli(["connection", "modify", conn_name,
                "wifi-sec.key-mgmt", "wpa-psk",
                "wifi-sec.psk", password,
                "connection.autoconnect", "no"])
    else:
        print(f"creating NetworkManager profile '{conn_name}'")
        _nmcli(["connection", "add",
                "type", "wifi",
                "con-name", conn_name,
                "ifname", device,
                "ssid", ssid,
                "wifi-sec.key-mgmt", "wpa-psk",
                "wifi-sec.psk", password,
                "ipv4.method", "auto",
                "connection.autoconnect", "no"])

    print(f"scanning for '{ssid}' on {device}...")

    timeout = 30.0
    deadline = time.monotonic() + timeout
    next_rescan = 0.0
    while time.monotonic() < deadline:
        # NM throttles rescans; spacing them ~10 s apart is the practical floor.
        if time.monotonic() >= next_rescan:
            _nmcli(["device", "wifi", "rescan", "ifname", device], check=False)
            next_rescan = time.monotonic() + 10.0
        if ssid in _visible_ssids(device):
            break
        time.sleep(0.5)
    else:
        visible = _visible_ssids(device)
        listing = ", ".join(sorted(visible)) if visible else "(no networks at all)"
        sys.exit(
            f"SSID '{ssid}' not visible after {timeout:.0f} s.\n"
            f"  {device} can currently see: {listing}\n"
            f"Likely causes:\n"
            f"  - the adapter is 5 GHz only (the ESP is on 2.4 GHz, channel 8)\n"
            f"  - the ESP isn't powered, or its credentials don't match\n"
            f"Manual check: `nmcli device wifi list ifname {device}`"
        )

    print(f"associating {device} → '{ssid}'")
    r = _nmcli(["-w", "20", "connection", "up", conn_name, "ifname", device],
               check=False)
    if r.returncode != 0:
        sys.exit(f"nmcli connection up → {r.stderr.strip() or r.stdout.strip()}")
    print(r.stdout.strip())


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
        faces.append([top_ci, i, (i + 1) % n])
    for i in range(n):
        faces.append([bot_ci, n + (i + 1) % n, n + i])
    for i in range(n):
        a, b = i, (i + 1) % n
        faces.append([a, n + a, n + b])
        faces.append([a, n + b, b])
    faces = np.array(faces, dtype=np.uint32)
    colors = np.tile(np.array(color, dtype=np.float32), (len(faces), 1))
    return verts, faces, colors


def _combine_meshes(*parts):
    vs, fs, cs, off = [], [], [], 0
    for v, f, c in parts:
        vs.append(v)
        fs.append(f + off)
        cs.append(c)
        off += len(v)
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


class FlightController:
    """Sends control packets to the ESP32 at 20 Hz.

    Held keys map to held axis values. Momentary commands (takeoff, land) set
    a flag for a single outgoing packet, then clear themselves — matching the
    pulse-trigger pattern from the legacy `drone.py`.
    """

    AXIS_KEYS = {"forward", "back", "left", "right", "up", "down", "turn"}

    # Bit positions in the FC `flags` byte (matches the legacy protocol).
    FLAG_FAST_FLY        = 0x01  # bit 0 — used here as "takeoff"
    FLAG_EMERGENCY_STOP  = 0x04  # bit 2 — kill motors (drone falls)
    FLAG_GYRO_CORR       = 0x40  # bit 6 — re-zero the FC's gyro to current pose

    def __init__(self, esp_ip, port):
        self._addr = (esp_ip, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pressed = set()
        self._pulse_flags = 0
        self._auto_descend = False
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    def press(self, key):
        if key not in self.AXIS_KEYS:
            return
        with self._lock:
            self._pressed.add(key)

    def release(self, key):
        with self._lock:
            self._pressed.discard(key)

    def takeoff(self):
        """Set the fast_fly bit on the next outgoing packet (single pulse)."""
        with self._lock:
            self._pulse_flags |= self.FLAG_FAST_FLY

    def emergency_stop(self):
        """Kill motors immediately (drone drops)."""
        with self._lock:
            self._pulse_flags |= self.FLAG_EMERGENCY_STOP

    def stabilize(self):
        """Reset to a known-good state: neutralise all axes, cancel the
        auto-descend latch, and tell the FC to re-zero its gyro using the
        current pose. Safe to use mid-flight as a "panic, hands off" button —
        the drone will drift, then the FC's own stabilisation takes over."""
        with self._lock:
            self._pressed.clear()
            self._auto_descend = False
            self._pulse_flags |= self.FLAG_GYRO_CORR

    def toggle_auto_descend(self):
        """Toggle a continuous 'down throttle' that doesn't need a key held."""
        with self._lock:
            self._auto_descend = not self._auto_descend

    def cancel_auto_descend(self):
        with self._lock:
            self._auto_descend = False

    def state(self):
        with self._lock:
            pressed = set(self._pressed)
            auto_descend = self._auto_descend
        pitch = roll = throttle = yaw = FC_NEUTRAL
        if "forward" in pressed: pitch    = FC_FULL_HIGH
        if "back"    in pressed: pitch    = FC_FULL_LOW
        if "right"   in pressed: roll     = FC_FULL_HIGH
        if "left"    in pressed: roll     = FC_FULL_LOW
        if "up"      in pressed: throttle = FC_FULL_HIGH
        elif "down"  in pressed or auto_descend:
                                 throttle = FC_FULL_LOW
        if "turn"    in pressed: yaw      = FC_FULL_HIGH
        return pitch, roll, throttle, yaw

    def _build_packet(self):
        pitch, roll, throttle, yaw = self.state()
        with self._lock:
            flags = self._pulse_flags
            self._pulse_flags = 0
        crc = pitch ^ roll ^ throttle ^ yaw ^ flags
        # On this FC the wire order is [roll, pitch, throttle, yaw], not the
        # [pitch, roll, …] suggested by the legacy variable names.
        return bytes([
            FC_PREFIX_CMD, FC_HEADER,
            roll, pitch, throttle, yaw,
            flags, crc, FC_TAIL,
        ])

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._sock.sendto(self._build_packet(), self._addr)
            except OSError as e:
                print(f"flight send error: {e}", file=sys.stderr)
            self._stop.wait(FLIGHT_INTERVAL)


class VideoReceiver(QtCore.QObject):
    """Receives fragmented JPEG frames over UDP and emits QImage on each completed frame.

    Wire format (per fragment):
        offset 0 : u32 frame_id (little-endian, monotonic)
        offset 4 : u16 packet_idx (0-based)
        offset 6 : u16 packet_total
        offset 8 : up to ~1400 bytes of JPEG payload
    """

    frame = QtCore.Signal(QtGui.QImage)

    def __init__(self, port):
        super().__init__()
        self._port = port
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        sock.bind(("0.0.0.0", self._port))
        sock.settimeout(1.0)

        # Pending frames: frame_id -> {"total": N, "parts": {idx: payload}}
        pending = {}
        latest = 0

        try:
            while not self._stop.is_set():
                try:
                    data, _ = sock.recvfrom(VIDEO_RECV_BUF)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if len(data) < VIDEO_HEADER_SIZE:
                    continue

                fid, idx, total = struct.unpack_from("<IHH", data, 0)
                payload = data[VIDEO_HEADER_SIZE:]

                entry = pending.setdefault(fid, {"total": total, "parts": {}})
                entry["parts"][idx] = payload

                if len(entry["parts"]) == entry["total"]:
                    parts = [entry["parts"][i] for i in range(entry["total"])]
                    jpeg = b"".join(parts)
                    img = QtGui.QImage.fromData(jpeg)
                    pending.pop(fid, None)
                    if not img.isNull():
                        self.frame.emit(img)

                # Garbage-collect frames we've moved past — drop anything
                # more than 3 frames behind the newest ID we've seen.
                if fid > latest:
                    latest = fid
                    pending = {k: v for k, v in pending.items() if k >= latest - 3}
        finally:
            sock.close()


class OrientationReceiver(QtCore.QObject):
    """Listens for 16-byte UDP quaternion frames on the local well-known port."""

    quat = QtCore.Signal(float, float, float, float)

    def __init__(self, port):
        super().__init__()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        self._sock.close()

    def _loop(self):
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

    KEY_AXIS = {
        QtCore.Qt.Key_Up:    "forward",
        QtCore.Qt.Key_Down:  "back",
        QtCore.Qt.Key_Left:  "left",
        QtCore.Qt.Key_Right: "right",
        QtCore.Qt.Key_U:     "up",
        QtCore.Qt.Key_D:     "down",
        QtCore.Qt.Key_A:     "turn",
    }

    @staticmethod
    def _keymap_legend():
        return ("↑↓ pitch   ←→ roll   U/D throttle   A yaw   "
                "T takeoff   S stabilize   L auto-descend   E e-stop")

    def _maybe_auto_tare(self):
        """Tare to the current orientation once both video and IMU are flowing."""
        if (self._tare is None
                and self._video_received and self._quat_received):
            self._tare = self._last_quat
            print("auto-tared (video + IMU streams ready)")

    def __init__(self, cfg):
        super().__init__()
        self.setWindowTitle(f"Drone viewer — {cfg['ESP_IP']}")
        self.resize(1024, 768)

        self._video = QtWidgets.QLabel("waiting for video…")
        self._video.setAlignment(QtCore.Qt.AlignCenter)
        self._video.setStyleSheet(
            "background-color: #000; color: #888; font-family: monospace;"
        )
        self.setCentralWidget(self._video)

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
        self._stats = QtWidgets.QLabel()
        self._stats.setStyleSheet("color: #888;")
        self._status.addPermanentWidget(self._stats)
        self._status.showMessage(self._keymap_legend())

        self._rx = OrientationReceiver(int(cfg["ORIENTATION_PORT"]))
        self._rx.quat.connect(self._on_quat)
        self._rx.start()
        print(f"orientation: listening on UDP port {cfg['ORIENTATION_PORT']}")

        self._video_rx = VideoReceiver(int(cfg["VIDEO_PORT"]))
        self._video_rx.frame.connect(self._on_video_frame)
        self._video_rx.start()
        print(f"video: listening on UDP port {cfg['VIDEO_PORT']}")

        self._fc = FlightController(cfg["ESP_IP"], int(cfg["FLIGHT_PORT"]))
        self._fc.start()
        print(f"flight control: udp://{cfg['ESP_IP']}:{cfg['FLIGHT_PORT']}")
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        self._frames = 0
        self._last_t = time.monotonic()
        self._last_quat = (1.0, 0.0, 0.0, 0.0)
        self._tare = None
        self._video_received = False
        self._quat_received = False
        self._last_video_pixmap = None
        self._video_frames = 0
        self._last_video_t = time.monotonic()
        self._video_fps = 0.0

    @staticmethod
    def _make_drone():
        BODY  = (0.32, 0.32, 0.10)
        ARM   = (0.55, 0.06, 0.04)
        MOTOR = (0.12, 0.12, 0.08)
        PROP_R, PROP_H = 0.20, 0.012

        DARK  = (0.15, 0.15, 0.18, 1)
        GRAY  = (0.40, 0.40, 0.45, 1)
        RED   = (0.85, 0.20, 0.20, 1)
        LIGHT = (0.55, 0.55, 0.60, 1)

        parts = [_box_mesh(BODY, color=DARK)]
        arm_len = ARM[0]

        arms = [
            ( math.pi / 4, RED),
            (-math.pi / 4, RED),
            ( 3 * math.pi / 4, GRAY),
            (-3 * math.pi / 4, GRAY),
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
        # Camera is mounted upside-down on the drone — flip vertically.
        self._last_video_pixmap = QtGui.QPixmap.fromImage(img.mirrored(False, True))
        self._refresh_video_pixmap()
        if not self._video_received:
            self._video_received = True
            self._maybe_auto_tare()
        self._video_frames += 1
        now = time.monotonic()
        if now - self._last_video_t >= 1.0:
            self._video_fps = self._video_frames / (now - self._last_video_t)
            self._video_frames = 0
            self._last_video_t = now

    @QtCore.Slot(float, float, float, float)
    def _on_quat(self, w, x, y, z):
        self._last_quat = (w, x, y, z)
        if not self._quat_received:
            self._quat_received = True
            self._maybe_auto_tare()
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
            self._stats.setText(
                f"imu {fps:.0f} Hz [{tare}]   video {self._video_fps:.0f} fps"
            )
            self._frames = 0
            self._last_t = now

    def keyPressEvent(self, ev):
        axis = self.KEY_AXIS.get(ev.key())
        if axis is not None:
            if not ev.isAutoRepeat():
                # Manual throttle input cancels the auto-descend latch.
                if axis in ("up", "down"):
                    self._fc.cancel_auto_descend()
                self._fc.press(axis)
            return
        if ev.isAutoRepeat():
            return
        k = ev.key()
        if k == QtCore.Qt.Key_T:
            # Re-tare immediately before takeoff so the rest pose reflects
            # whatever orientation the drone is sitting in right now.
            self._tare = self._last_quat
            self._fc.takeoff()
        elif k == QtCore.Qt.Key_S:
            self._fc.stabilize()
        elif k == QtCore.Qt.Key_L:
            self._fc.toggle_auto_descend()
        elif k == QtCore.Qt.Key_E:
            self._fc.emergency_stop()
        else:
            super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev):
        axis = self.KEY_AXIS.get(ev.key())
        if axis is not None:
            if not ev.isAutoRepeat():
                self._fc.release(axis)
            return
        super().keyReleaseEvent(ev)

    def closeEvent(self, ev):
        self._rx.stop()
        self._video_rx.stop()
        self._fc.stop()
        super().closeEvent(ev)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("wifi_device",
                        help="WiFi interface to associate with the drone AP (e.g. wlp3s0)")
    parser.add_argument("--credentials",
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             "..", "main", "wifi_credentials.h"),
                        help="path to wifi_credentials.h (defaults to ../main/wifi_credentials.h)")
    parser.add_argument("--no-connect", action="store_true",
                        help="skip nmcli connect; assume the device is already on the AP")
    args = parser.parse_args()

    cfg = parse_credentials(args.credentials)

    if not args.no_connect:
        connect_to_ap(args.wifi_device, cfg["WIFI_SSID"], cfg["WIFI_PASSWORD"])

    app = QtWidgets.QApplication(sys.argv)
    win = Viewer(cfg)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
