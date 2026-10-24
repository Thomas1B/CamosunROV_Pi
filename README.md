# CamosunROV_Pi

Raspberry Pi 4 software for the CamosunROV underwater vehicle.

The Pi sits in the ROV between the surface command centre and the STM32F446RE. It:

- receives control commands from the command centre over the tether (UDP),
- forwards them to the STM32 at 50 Hz over UART, which drives the thrusters and camera tilt servo,
- forwards the Bar30 reference values (surface pressure, water density, second-order calibration) from the command centre to the STM32,
- forwards the STM32's telemetry (depth, temperatures, battery, attitude, leak, faults) back to the command centre as JSON,
- streams live H.264 video from the Pi camera to the command centre.

It works together with **CamosunROV_STM32** (motor control and sensors) and **CamosunROV_CommandCentre** (the laptop console).

---

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi 4 | Raspberry Pi OS (Trixie), Lite or Desktop, user `pirov`, repo at `/home/pirov/CamosunROV_Pi` |
| STM32F446RE | Runs CamosunROV_STM32. Connected to the Pi's PL011 UART (`/dev/serial0`) |
| Camera | Camera Module v2 (IMX219) now, HQ Camera (IMX477) later. No code changes needed to swap |
| Tether | Blue Robotics Fathom tether, RJ45 on the Pi's `eth0` and on the command centre |
| Battery | Blue Robotics Li-ion 4S 18 Ah (14.8 V nominal) |

**UART wiring (3.3 V logic on both sides, 115 200 baud, 8N1):**

| Raspberry Pi 4 | | STM32F446RE |
|---|---|---|
| GPIO14 / TXD (header pin 8) | → | UART RX |
| GPIO15 / RXD (header pin 10) | ← | UART TX |
| GND (header pin 6) | — | GND |

---

## Setup

When flashing the SD card in Raspberry Pi Imager, set the **username to `pirov`**. The first line of `main.py` and `Test_UART_DMA.py` points to `/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3`, so with any other username, or with the repo cloned somewhere else, `./main.py` fails with "bad interpreter". The hostname can be anything; nothing in the code uses it. The password can be anything too, but remember it: `setup_pi.sh` asks for it once so it can use `sudo`.

Then, on the fresh Raspberry Pi OS install (Lite or Desktop), logged in as `pirov` and connected to the internet over Wi-Fi:

```bash
cd ~
git clone <repo-url> CamosunROV_Pi
cd CamosunROV_Pi
bash setup_pi.sh          # run as your normal user, NOT with sudo
```

`setup_pi.sh` checks each step and skips anything already done, so it is safe to run again after pulling new code. It:

1. Installs system packages: `git`, `python3-venv`, `rpicam-apps-lite`, `python3-picamera2`, `python3-libcamera`, `python3-av`
2. Configures the UART: serial console off, UART on, Bluetooth off (`dtoverlay=disable-bt`) so `/dev/serial0` is the full PL011 UART
3. Adds the user to the `dialout` (serial) and `video` (camera) groups
4. Creates the `rov-tether` NetworkManager profile: `eth0` = `192.168.2.2/24`
5. Creates the `ROV_venv` virtual environment (with system site packages, so it uses the apt camera stack) and installs `pyserial`
6. Makes `main.py` and `Test_UART_DMA.py` executable and strips Windows line endings from their first line
7. Disables services the ROV does not need, for a faster boot
8. Installs and enables the `rov` systemd service so `main.py` starts at every power-on

It then asks whether to reboot. **A reboot is required** the first time for the UART and group changes to take effect.

The command centre side (laptop IP and firewall) is covered in the CamosunROV_CommandCentre README.

---

## Running

**You don't need to run anything.** After setup, the ROV software starts by itself every time the Pi powers on. It runs as a background service called `rov`.

### Optional: checking on the service

You only need these commands if you want to check on the service or control it. None of them are part of normal use.

| If you want to… | Run |
|---|---|
| See whether it's running | `systemctl status rov` |
| Watch its live output | `journalctl -u rov -f` |
| Apply changes after editing code or settings | `sudo systemctl restart rov` |
| Stop it from starting at boot | `sudo systemctl disable rov` |
| Stop it so you can run it by hand (below) | `sudo systemctl stop rov` |

### Optional: running it by hand

This is useful for debugging, when you want to see the output directly in your terminal. Stop the service first:

```bash
sudo systemctl stop rov   # only one program can use /dev/serial0 and UDP 5600
cd ~/CamosunROV_Pi
./main.py                 # Ctrl+C to stop; sends the STM32 a stop frame on exit
```

Every console line is tagged with the process it came from:

```
[MAIN]   CamosunROV Pi starting -- Ctrl+C to stop
[BRIDGE] Started: UDP 5600 in / 5601 out  <->  /dev/serial0 @ 115200 baud
[CAMERA] Started
[BRIDGE] *** Connected to STM32 -- telemetry received (waited 0.1 s) ***
[CAMERA] *** Streaming: imx219  1920x1080 @ 30 fps, 4.0 Mbit/s -> 192.168.2.1:5602 ***
[BRIDGE] *** Connected to command centre at 192.168.2.1 -- commands received ... ***
[BRIDGE] [    52] Pressure: 101.3 kPa  Depth: 0.02 m  Water: 12.40 C  Battery: 16.10 V  ...
[BRIDGE]          Command: ARMED     Motors: [0, 0, 20, 20, 0, 0] %  Tilt: +0 deg  (packets 812, bad 0)
```

Telemetry and command lines print at 2 Hz. Leak and fault changes, arm and disarm, and connect and disconnect events print straight away.

### How `main.py` works

`main.py` runs the **bridge** and the **camera** as two separate processes. A camera crash or hang can never stop thruster control, and video encoding cannot disturb the 50 Hz UART timing. If either process stops, `main.py` restarts it after 2 s, doubling the delay up to 10 s if it keeps failing. The other process keeps running.

### Testing the STM32 link alone

`Test_UART_DMA.py` talks to the STM32 directly with no command centre or camera. It sends zero-thrust frames at 50 Hz and prints the telemetry.

```bash
sudo systemctl stop rov
./Test_UART_DMA.py
```

While it is running, type `r` + Enter to send a leak-recover command, or `q` + Enter to quit.

---

## Configuration

All settings are in [`rov/settings.py`](rov/settings.py). After changing one, restart with `sudo systemctl restart rov`.

| Setting | Default | Meaning |
|---|---|---|
| `LAPTOP_IP` | `192.168.2.1` | Where the video stream is sent |
| `CMD_PORT` | 5600 | Commands, laptop → Pi |
| `TELEM_PORT` | 5601 | Telemetry, Pi → laptop |
| `VIDEO_PORT` | 5602 | Video, Pi → laptop |
| `SERIAL_PORT` / `BAUD` | `/dev/serial0` / 115 200 | UART to the STM32 |
| `UART_PERIOD` | 0.02 s (50 Hz) | Command frame rate to the STM32 |
| `CMD_TIMEOUT` | 0.25 s | No valid command for this long → thrusters 0 % |
| `STM32_TIMEOUT` | 0.5 s | No valid telemetry for this long → STM32 treated as disconnected |
| `HEARTBEAT_PERIOD` | 0.1 s | Telemetry period to the laptop while the STM32 is silent |
| `REF_UART_REPEATS` | 3 | Each new set of Bar30 reference values is sent to the STM32 this many times, and again whenever the STM32 reconnects |
| `VIDEO_SIZE` | 1920 × 1080 px | Camera resolution |
| `FPS` | 30 frame/s | Camera frame rate |
| `BITRATE` | 4 Mbit/s | H.264 bit rate (~4 % of the 100 Mbit/s tether link) |
| `IPERIOD` | 15 frames | Interval between I-frames (0.5 s): faster recovery after packet loss |
| `HFLIP` / `VFLIP` | `False` | Mirror or flip the image to suit how the camera is mounted |

---

## Protocols

### Command packet: command centre → Pi

UDP to port 5600, 16 bytes, little-endian, `struct` format `<2sBBH6bbxH`. Must match `command_center_py/link.py`.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `2s` | signature | `b"RV"`. Identifies the packet as a CamosunROV command; anything else is ignored |
| 2 | `u8` | version | `1`. Packet layout version. Raise it on both sides whenever the layout changes; a mismatched packet is dropped instead of being read wrong |
| 3 | `u8` | flags | bit 0 = armed, bit 1 = recover (clear a leak shutdown) |
| 4 | `u16` | seq | Packet counter, +1 per packet, wraps to 0 after 65 535. The Pi echoes it back as `ack` in the telemetry so the command centre can confirm commands are arriving |
| 6 | `6 × i8` | throttle | T1 … T6, −100 … +100 % |
| 12 | `i8` | cam_tilt | degrees, 0 = level, + = up |
| 13 | — | pad | Unused, always 0. Keeps the CRC on an even offset and leaves a spare byte for a future field |
| 14 | `u16` | crc16 | CRC-16/CCITT-FALSE over bytes 0 … 13 |

Packets with the wrong length, signature, version or CRC are dropped. When the packet says disarmed, the thrusters are sent to the STM32 as 0 %; the camera tilt still follows the command.

### Reference packet: command centre → Pi

The Bar30 reference values set in the command centre's CONNECT dialog or **File ▸ Reference Values…**. UDP to the same port as the command packet (5600), told apart by its signature. 16 bytes, little-endian, `struct` format `<2sBBHffH`. Must match `command_center_py/link.py`.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | `2s` | signature | `b"RF"` (commands are `b"RV"`) |
| 2 | `u8` | version | `1` |
| 3 | `u8` | flags | bit 0 = second-order pressure calibration on (water below 20 °C) |
| 4 | `u16` | ref_seq | +1 each time the values change in the command centre |
| 6 | `f32` | surface | Surface pressure reference, hPa (= mbar) |
| 10 | `f32` | density | Water density, kg/m³ |
| 14 | `u16` | crc16 | CRC-16/CCITT-FALSE over bytes 0 … 13 |

Malformed or out-of-range packets (surface outside 800 … 1100 hPa, density outside 900 … 1100 kg/m³) are dropped. A new `ref_seq` is forwarded to the STM32 as a reference frame (below) and then reported back as `ref_ack` in the telemetry. The command centre resends the packet every second until it sees that `ack`, so the values still arrive if the STM32 connects later or the Pi restarts. They are also re-sent to the STM32 whenever it reconnects.

### Telemetry: Pi → command centre

UDP to port 5601 at the IP address of the most recent valid command packet. One JSON object per datagram, sent at 50 Hz (one per STM32 frame):

```json
{"depth_m": 1.23, "pressure_kpa": 113.6, "water_c": 8.70, "internal_c": 31.2,
 "roll": 1.20, "pitch": -0.40, "yaw": 214.0, "voltage": 15.80,
 "leak": false, "faults": ["TEMP"], "fault_flags": 8, "stm32": true,
 "uart_errors": 0, "ack": 1234, "rx_cmds": 5678, "ref_ack": 3}
```

Units: m, kPa, °C, degrees, V.

- When a sensor's fault bit is set, its values are sent as `null`, because the STM32 is holding a stale reading.
- When the STM32 is not answering, `"stm32"` is `false`, every sensor value is `null`, and heartbeats are sent at 10 Hz. That way the command centre can still tell that the tether and the Pi are working.
- `ack` is the last command `seq` received, `rx_cmds` is the total number of valid commands, and `uart_errors` is the number of STM32 frames lost to checksum failures.
- `ref_ack` is the `ref_seq` of the reference values last forwarded to the STM32 (`null` until the command centre has sent any).

### UART frames: Pi ⇄ STM32

The frame format is defined in [`rov/UART_DMA.py`](rov/UART_DMA.py) and documented in the CamosunROV_STM32 README. `UART_DMA.py` **must be kept in sync with `UART_DMA.h/.c`** on the STM32 side.

| Frame | Direction | Size | Start byte | When |
|---|---|---|---|---|
| Command | Pi → STM32 | 11 bytes | `0xAA` | Every 20 ms (50 Hz) |
| Reference | Pi → STM32 | 10 bytes | `0xAB` | When the Bar30 reference values change (3 times), and when the STM32 reconnects |
| Telemetry | STM32 → Pi | 20 bytes | `0xAA` | Every 20 ms |

**Reference frame** (`START_REF | surface_hpa_x100 (u32) | density_x10 (u16) | flags (u8) | ref_id (u8) | CHECKSUM`):

- `surface_hpa_x100` is the surface pressure in hPa × 100 (e.g. 101325 = 1013.25 hPa). It replaces the firmware's `SURFACE_PRESSURE`.
- `density_x10` is the water density in kg/m³ × 10 (e.g. 10000 = 1000.0). It replaces `WATER_DENSITY`.
- `flags` bit 0 turns the second-order calibration on. It replaces `second_order_calculation`.
- `ref_id` goes up by 1 with each new set of values, so the STM32 can ignore the repeats.
- The checksum is the same additive checksum as the other frames.

It is written half a period (10 ms) after a command frame, in its own `write()`, so the STM32's idle-line DMA sees it as a separate frame and the 50 Hz command frames are not disturbed. **The STM32 firmware needs to be updated to use it.** The frame is deliberately shorter than a command frame: the current firmware only scans chunks of at least 11 bytes, so it ignores the frame and keeps using its `#define` values.

### Video

The Pi camera feed is H.264, encoded on the Pi 4's hardware encoder and sent as MPEG-TS over UDP to `LAPTOP_IP:5602`. SPS and PPS are repeated on every I-frame, so the laptop can join the stream at any time.

---

## Safety behaviour

- **Command failsafe:** if no valid command packet arrives for 0.25 s (for example, the tether is unplugged or the laptop crashes), the Pi sends 0 % to all thrusters and holds the current tilt.
- **Leak shutdown:** the STM32 cuts the thrusters and the tilt servo. It stays shut down until the sensor is dry *and* the command centre sends a recover command.
- **Clean exit:** Ctrl+C or `systemctl stop rov` sends three stop frames (thrusters 0 %, camera level) before the UART closes.
- **Process isolation:** the bridge and the camera run as separate processes (`multiprocessing`), so a camera crash or hang cannot affect thruster control.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| `[BRIDGE] !!! Cannot open /dev/serial0` | UART not enabled (re-run `setup_pi.sh` and reboot), user not in `dialout`, or `Test_UART_DMA.py` is running |
| `[BRIDGE] !!! UDP port 5600 is already in use` | Another copy is running: `sudo systemctl stop rov` |
| `Waiting for connection to STM32...` | STM32 power, UART wiring (TX ↔ RX crossed, common GND), STM32 firmware running |
| `Waiting for command centre...` | `ping 192.168.2.2` from the laptop; check the IP/port in the console's CONNECT dialog |
| One sensor reads `null` / `--` | That sensor's fault bit is set; see `faults` in the telemetry |
| No video | Read the `[CAMERA]` lines. Run `rpicam-hello --list-cameras` |
| `bad interpreter: ...^M` | Windows line endings: `sed -i 's/\r$//' main.py` (`setup_pi.sh` also fixes this) |
| Image mirrored or upside down | `HFLIP` / `VFLIP` in `rov/settings.py` |
