# Usage

## ESP32 firmware

### First-time setup
1. Copy WiFi credentials template and fill in:
   ```bash
   cp main/wifi_credentials.h.example main/wifi_credentials.h
   $EDITOR main/wifi_credentials.h
   ```
2. Set the chip target (once per checkout):
   ```bash
   idf.py set-target esp32s3
   ```

### Build, flash, monitor
```bash
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

On boot you should see:
- `ICM-20948 detected`
- `DMP enabled (Quat9 / 9-axis)`
- `camera ready (VGA JPEG)`
- `wifi: got IP …`
- `Orientation: POST http://<ip>/subscribe  {"port":N}`
- `Video:       GET  http://<ip>:81/stream`

If the WHO_AM_I check loops, see the I²C section in `AGENTS.md` (external 4.7 kΩ pull-ups on SDA/SCL → 3V3).

---

## Python viewer

### NixOS (preferred)

```bash
cd client
nix-shell
python imu_viewer.py <esp32-ip>
```

`shell.nix` pulls `numpy`, `pyqtgraph`, `pyside6`, `pyopengl`, and `requests` from nixpkgs. Don't use pip on NixOS — wheels expect FHS paths and fail with `libz.so.1: cannot open shared object file`.

### Other systems (venv + pip)

```bash
cd client
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python imu_viewer.py <esp32-ip>
```

### Options
- `--port N` — local UDP port to bind (default `9000`, set to `0` for ephemeral).

### Keyboard shortcuts (in the viewer window)
- **T** — tare: capture the current quaternion as the rest pose, so the box appears at identity in that orientation. Use this to compensate for the DMP's mounting-matrix baseline.
- **R** — reset tare: revert to raw DMP output.

The script subscribes to the ESP32 (`POST /subscribe`) every 2 s; subscriptions expire after 5 s on the ESP32 side, so a crashed client stops getting packets automatically.

---

## Firewall

The viewer binds UDP/9000 by default; the ESP32 pushes datagrams to it. Most distros block incoming UDP — open the port before running.

### Temporary (NixOS, session-only)

```bash
sudo iptables -I nixos-fw -p udp --dport 9000 -j ACCEPT
# remove afterwards:
sudo iptables -D nixos-fw -p udp --dport 9000 -j ACCEPT
```

Tighter scope (only allow the ESP32 itself):

```bash
sudo iptables -I nixos-fw -p udp -s 192.168.1.161 --dport 9000 -j ACCEPT
```

If your distro doesn't have the `nixos-fw` chain, use `INPUT` instead.

### Persistent (NixOS)

Add to `/etc/nixos/configuration.nix`:

```nix
networking.firewall.allowedUDPPorts = [ 9000 ];
```

Then `sudo nixos-rebuild switch`. The temporary iptables rule is no longer needed.

---

## Critical configuration (don't regress these)

A few non-obvious choices are load-bearing — if any of them gets reverted, the
firmware boot-loops, the magnetometer fusion stops, or one of the endpoints
locks up.

1. **External pull-ups on I²C** (4.7 kΩ from SDA/SCL → 3V3). The XIAO's
   internal pulls and the ICM-20948 breakout's 10 kΩ alone are too weak under
   WiFi RF noise — the bus eventually returns `ESP_ERR_INVALID_STATE` and
   never recovers. With the externals, 400 kHz is rock-solid.

2. **`icm20948_i2c_master_enable(true)` before the DMP setup** (in
   `main/main.c::imu_setup`). The cybergear lib's
   `init_dmp_sensor_with_defaults` configures SLV0/SLV1 to read the AK09916
   but never enables the chip's internal I²C master, so the magnetometer is
   silently absent and Quat9 fusion runs as 6-axis (yaw drifts forever).

3. **Camera SCCB on I²C port 1 with the legacy driver** (sdkconfig.defaults:
   `CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y`,
   `CONFIG_SCCB_HARDWARE_I2C_PORT1=y`). The IMU uses the legacy driver on
   port 0; if the camera tries the new (`driver_ng`) driver on the same port
   the chip hard-aborts at boot (`CONFLICT! driver_ng is not allowed to be
   used with this old driver`). Same driver kind, different ports.

4. **Two HTTP servers, on ports 80 and 81.** `/stream` is an MJPEG handler
   that never returns; if it lives on the same `httpd` as `/subscribe`, the
   single worker is stuck forever and `POST /subscribe` times out. The
   firmware boots a second httpd on port 81 just for video — keep it that way.

5. **PSRAM + 8 MB flash defaults** (sdkconfig.defaults: `CONFIG_SPIRAM=y`,
   `CONFIG_SPIRAM_MODE_OCT=y`, `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`). The camera
   framebuffers go in PSRAM and won't fit anywhere else; flash size has to
   match the chip or the bootloader truncates.

6. **`espressif/esp32-camera` is pulled via the IDF component manager**
   (`main/idf_component.yml`). On rebuild, it lands in
   `managed_components/espressif__esp32-camera/` (gitignored). Keep
   `dependencies.lock` gitignored too — re-resolve on each clone.

## API

### `POST /subscribe`

Request body:
```json
{"port": 9000}
```

The ESP32 captures the caller's IP from the TCP connection and registers
`(ip, port)` for UDP push. Up to 4 subscribers; entries expire 5 s after the
last refresh, so clients should re-POST every 2 s.

Response: `{"ok": true}` on success.

### `GET /stream` (on port 81)

MJPEG video stream over chunked HTTP (`Content-Type: multipart/x-mixed-replace; boundary=frame`). Each part is a complete JPEG frame at VGA (640×480) resolution, ~20 fps depending on lighting and WiFi bandwidth. Served on a **separate HTTP server on port 81** so the never-returning streaming handler doesn't block control endpoints on port 80. Open in any MJPEG-capable client (VLC, browser, `ffplay`, the bundled Python viewer):

```bash
ffplay http://<ip>:81/stream
```

### UDP push payload

16 bytes, little-endian, sent to each active subscriber whenever the DMP
produces a new Quat9 frame:

| Offset | Type     | Field |
|--------|----------|-------|
| 0      | float32  | w     |
| 4      | float32  | x     |
| 8      | float32  | y     |
| 12     | float32  | z     |

The quaternion is unit-normalised (within numerical error). On the wire it
arrives at the DMP's natural rate (~55 Hz with the default ODR setting).
