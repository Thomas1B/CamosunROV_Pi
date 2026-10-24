"""
All the ROV's tunable settings in one place: network, UART, rates, camera.
Change values here, then restart main.py (or: sudo systemctl restart rov).

SI units throughout: s, Hz, px, bit/s, deg.
"""

# ---------------- network (tether) ----------------
# Pi eth0 = 192.168.2.2/24, command centre laptop = 192.168.2.1/24
LAPTOP_IP = "192.168.2.1"      # where the camera stream is sent
CMD_PORT = 5600                # laptop -> Pi commands (must match the command centre profile)
TELEM_PORT = 5601              # Pi -> laptop telemetry (sent to whichever IP the commands come from)
VIDEO_PORT = 5602              # Pi -> laptop camera stream

# ---------------- UART to the STM32 ----------------
SERIAL_PORT = "/dev/serial0"   # PL011 UART: dtoverlay=disable-bt in config.txt
BAUD = 115200

# ---------------- bridge timing ----------------
UART_PERIOD = 1 / 50           # s   -> command frames to the STM32 at 50 Hz
CMD_TIMEOUT = 0.25             # s   without a valid command packet -> thrusters zeroed
STM32_TIMEOUT = 0.5            # s   without valid telemetry -> STM32 treated as not connected
                               #     (telemetry normally arrives every 0.02 s, so 0.5 s = 25 missed frames)
HEARTBEAT_PERIOD = 0.1         # s   telemetry rate to the laptop while the STM32 is silent
REF_UART_REPEATS = 3           # each new set of Bar30 reference values is sent to the STM32 this many
                               # times (one per UART period), and again whenever the STM32 reconnects

# ---------------- console printing (same style as Test_UART_DMA.py) ----------------
PRINT_PERIOD = 1 / 2           # s   telemetry + command lines printed at 2 Hz
WAIT_PRINT_PERIOD = 2.0        # s   between "Waiting for ..." messages

# ---------------- camera ----------------
VIDEO_SIZE = (1920, 1080)       # px  (v2 camera now, HQ camera later: picamera2 picks a matching mode)
FPS = 30                       # frames/s
BITRATE = 4_000_000            # bit/s -- ~4 % of the 100 Mbit/s tether link
IPERIOD = 15                   # frames between I-frames (0.5 s at 30 fps): faster recovery from loss
HFLIP = True                  # set True if the image is mirrored once mounted
VFLIP = True                  # set True (with HFLIP) if the camera is mounted upside down
CAMERA_RETRY_DELAY = 2.0       # s   before reopening the camera after an error
CAMERA_START_TIMEOUT = 5.0     # s   no first frame after starting -> restart the camera process
                               #     (the HQ camera can fail to start at boot without any error)
CAMERA_STALL_TIMEOUT = 3.0     # s   frames stopped mid-stream -> restart the camera process

# ---------------- main.py supervisor ----------------
RESTART_DELAY_MIN = 2.0        # s   before restarting a part that stopped
RESTART_DELAY_MAX = 10.0       # s   longest wait if it keeps failing (doubles each time)
