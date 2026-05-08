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
- `wifi: got IP …`
- `Subscribe at: POST http://<ip>/subscribe  body: {"port":N}`

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
