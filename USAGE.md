# Usage

## ESP32 firmware

### First-time setup
1. Copy the WiFi credentials template and pick the SSID/PSK for the drone's
   own access point (the ESP32 is the AP — there's no upstream router).
   Other fields rarely need changing:
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
- `wifi_ap: AP 'esp32-drone' up on channel 1, ESP=192.168.4.1, client=192.168.4.2`
- `camera ready (VGA JPEG)`
- `AP 'esp32-drone' ready (ESP=192.168.4.1, client=192.168.4.2)`
- `Orientation:    UDP  192.168.4.1 -> 192.168.4.2:9000`
- `Video:          UDP  192.168.4.1 -> 192.168.4.2:9001`
- `Flight control: UDP  192.168.4.2 -> 192.168.4.1:7099`

If the WHO_AM_I check loops, see the I²C section in `AGENTS.md` (external 4.7 kΩ pull-ups on SDA/SCL → 3V3).

---

## Python viewer

The viewer takes the WiFi interface as its only required argument. It then:
1. reads SSID/PSK/IPs/ports from `../main/wifi_credentials.h`,
2. associates the given interface with the drone's AP via `nmcli`,
3. opens the orientation/flight/video sockets — all destinations are
   already known from the credentials file, no handshake required.

### NixOS (preferred)

```bash
cd client
nix-shell
python imu_viewer.py <wifi-device>      # e.g. wlp3s0
```

`shell.nix` pulls `numpy`, `pyqtgraph`, `pyside6`, `pyopengl`, and `requests` from nixpkgs. Don't use pip on NixOS — wheels expect FHS paths and fail with `libz.so.1: cannot open shared object file`.

### Other systems (venv + pip)

```bash
cd client
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python imu_viewer.py <wifi-device>
```

### Options
- `--credentials <path>` — alternate `wifi_credentials.h` (default: `../main/wifi_credentials.h`).
- `--no-connect` — skip the `nmcli` step; assume the device is already on the AP.

### Keyboard shortcuts (in the viewer window)

Flight control (hold-to-move; release returns the axis to neutral):
- **↑ / ↓** — pitch forward / backward
- **← / →** — roll left / right
- **U / D** — throttle up / down
- **A** — yaw (turn around)

Momentary commands (single-packet pulse):
- **T** — takeoff. Re-tares the 3D orientation reference to the current
  pose **before** sending the takeoff packet, so the on-screen drone
  matches the physical "ready to fly" attitude.
- **E** — emergency stop (sets the `emergency_stop` bit — kills motors).

Trim (drift correction):
- **Shift + ↑ / ↓** — adjust pitch trim (forward / backward bias)
- **Shift + ← / →** — adjust roll trim (left / right bias)

Each press shifts the corresponding axis baseline by 5 (clamped to ±60).
The trim is added to whatever you're commanding, so it acts as a constant
offset that compensates for the drone's natural drift. Current values are
shown in the bottom-right stats panel as `trim P=+5 R=-3`. Use them when
the drone slowly drifts in one direction even with no keys pressed.

Latched commands (toggle on/off):
- **L** — auto-descend: keeps the throttle pinned low without holding **D**.
  Press again to stop, or press **U** / **D** to take manual control (which
  cancels the latch).

Tare is also applied **automatically** the moment the viewer first sees
both the video stream and the IMU stream — so the rest pose is captured
when the drone is sitting still on the ground at startup. Pressing **T**
just refreshes that reference at takeoff time.

### Object detection overlay

If `ultralytics` is installed (`pip install ultralytics`, or in `shell.nix`
on NixOS), the viewer runs YOLOv8n on every video frame in a background
thread and overlays bounding boxes for the **80 COCO classes** — people,
animals, vehicles, common objects. Each class gets a stable colour (golden-
angle hue stepping) and a confidence label.

**First-run note**: ultralytics auto-downloads `yolov8n.pt` (~6 MB) to
`~/.cache/ultralytics/` the first time. **Do this while you still have
internet** — once you're associated with the drone AP, you're offline and
the download will fail. Pre-warm with:

```bash
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

If the model can't be loaded, the viewer just shows plain video — no
crash. Look for `detector: YOLOv8n ready` (success) or
`detector disabled: …` (fallback) in the startup output.

The script subscribes to the ESP32 (`POST /subscribe`) every 2 s; subscriptions expire after 5 s on the ESP32 side, so a crashed client stops getting packets automatically.

---

## Firewall

NetworkManager assigns the WiFi device to the drone AP's subnet
(`192.168.4.0/24`). The viewer binds the orientation port (UDP/9000 by
default) on that interface to receive quaternions from the ESP. Most
distros block incoming UDP unless you punch a hole.

The viewer needs **two** UDP ports open: orientation (`9000`) and video (`9001`).

### Temporary (NixOS, session-only)

```bash
sudo iptables -I nixos-fw -p udp -m multiport --dports 9000,9001 -j ACCEPT
# remove afterwards:
sudo iptables -D nixos-fw -p udp -m multiport --dports 9000,9001 -j ACCEPT
```

Tighter scope (only accept traffic from the ESP):

```bash
sudo iptables -I nixos-fw -p udp -s 192.168.4.1 \
    -m multiport --dports 9000,9001 -j ACCEPT
```

If your distro doesn't have the `nixos-fw` chain, use `INPUT` instead.

### Persistent (NixOS)

Add to `/etc/nixos/configuration.nix`:

```nix
networking.firewall.allowedUDPPorts = [ 9000 9001 ];
```

Then `sudo nixos-rebuild switch`. The temporary iptables rule is no longer needed.

---

## Critical configuration (don't regress these)

A few non-obvious choices are load-bearing — if any of them gets reverted, the
firmware boot-loops, the magnetometer fusion stops, or one of the endpoints
locks up.

1. **Strong I²C pull-ups (~2 kΩ effective).** The XIAO's internal pulls and a
   weak 10 kΩ on the breakout are too high under WiFi RF noise — the bus
   eventually returns `ESP_ERR_INVALID_STATE` and never recovers. The GY-912
   board ships with 2.2 kΩ pulls already wired to its 3V3 rail and is fine on
   its own. Older ICM-20948 breakouts (10 kΩ on board) need 4.7 kΩ externals
   in parallel.

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

4. **No HTTP server.** Everything is UDP — orientation, video, control.
   Earlier versions had an MJPEG `/stream` handler on its own httpd; that
   was an infinite loop that monopolised one of LWIP's scarce TCP slots.
   Going pure-UDP also removes the `esp_http_server` dependency.

5. **PSRAM + 8 MB flash defaults** (sdkconfig.defaults: `CONFIG_SPIRAM=y`,
   `CONFIG_SPIRAM_MODE_OCT=y`, `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`). The camera
   framebuffers go in PSRAM and won't fit anywhere else; flash size has to
   match the chip or the bootloader truncates.

6. **`espressif/esp32-camera` is pulled via the IDF component manager**
   (`main/idf_component.yml`). On rebuild, it lands in
   `managed_components/espressif__esp32-camera/` (gitignored). Keep
   `dependencies.lock` gitignored too — re-resolve on each clone.

7. **Console moved to USB-Serial-JTAG** (sdkconfig.defaults:
   `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`). This frees GPIO 43 (XIAO D6)
   for use as UART1 TX to the flight controller. `/dev/ttyACM0` continues
   to carry the log stream as before — same wire, different peripheral.

## API

### Video UDP payload

Each fragment is sent to `CLIENT_IP:VIDEO_PORT` with an 8-byte little-endian header followed by up to 1400 bytes of JPEG. The client groups fragments by `frame_id` and emits the JPEG once all `packet_total` fragments have arrived; if a fragment is lost the whole frame is dropped (next one is ~50 ms away).

| Offset | Type     | Field          |
|--------|----------|----------------|
| 0      | uint32   | `frame_id` (monotonic) |
| 4      | uint16   | `packet_idx` (0-based) |
| 6      | uint16   | `packet_total` |
| 8      | bytes    | JPEG payload   |

### Flight control (UDP port 7099)

The ESP32 listens on UDP/7099 and forwards each datagram (after stripping the first byte) to UART1 TX (GPIO 43, 19200 8N1) where the original flight controller is wired. Two packet shapes:

**Control frame** — 9 bytes UDP, 8 forwarded to UART:

| Byte | Meaning |
|------|---------|
| 0    | `0x03` framing prefix (stripped) |
| 1    | `0x66` header |
| 2    | pitch (0–255, 128 = neutral) |
| 3    | roll  (0–255, 128 = neutral) |
| 4    | throttle (0–255) |
| 5    | yaw   (0–255, 128 = neutral) |
| 6    | flag bitmap (fast_fly, fast_drop, e-stop, …) |
| 7    | CRC = `pitch ^ roll ^ throttle ^ yaw ^ flags` |
| 8    | `0x99` tail |

**Heartbeat** — the ESP32 replies with `48 01 00 00 00` every 50 ms once a client has been seen, addressed to the latest sender. The client uses this as a link-alive signal.

### Orientation UDP payload

16 bytes, little-endian, pushed to `CLIENT_IP:ORIENTATION_PORT` (default
`192.168.4.2:9000`) whenever the DMP produces a new Quat9 frame:

| Offset | Type     | Field |
|--------|----------|-------|
| 0      | float32  | w     |
| 4      | float32  | x     |
| 8      | float32  | y     |
| 12     | float32  | z     |

The quaternion is unit-normalised (within numerical error). On the wire it
arrives at the DMP's natural rate (~55 Hz with the default ODR setting).
