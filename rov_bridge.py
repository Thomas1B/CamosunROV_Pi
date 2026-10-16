#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3
"""
ROV bridge: command centre (Ethernet/UDP) <-> Raspberry Pi <-> STM32 (UART).

    laptop --UDP 5600, 16-byte command packet, 50 Hz--> Pi --UART 11-byte frame, 50 Hz--> STM32
    laptop <--UDP 5601, JSON telemetry, 50 Hz---------- Pi <--UART 20-byte frame, 50 Hz-- STM32

The UART side reuses UART_DMA.py unchanged (the same code Test_UART_DMA.py
uses). Only one program can use /dev/serial0 at a time, so stop this bridge
before running Test_UART_DMA.py, and the other way round.

-----------------------------------------------------------------------------
COMMAND PACKET  laptop -> Pi  (matches command_center_py/link.py)
    UDP, CMD_PORT, 16 bytes, little-endian, struct "<2sBBH6bbxH"
    0  2s  magic     b"RV"
    2  B   version   1
    3  B   flags     bit0 = armed, bit1 = recover (clear a leak shutdown)
    4  H   seq       +1 every packet, wraps at 65535
    6  6b  throttle  T1..T6, -100..+100 %
    12 b   cam_tilt  degrees, 0 = level, + = up
    13 x   pad
    14 H   crc16     CRC-16/CCITT-FALSE over bytes 0..13 (binascii.crc_hqx(pkt[:14], 0xFFFF))

    Packets with the wrong length, magic, version or CRC are dropped.
    Disarmed (bit0 = 0) -> thrusters are sent to the STM32 as 0 %, tilt still follows.

TELEMETRY  Pi -> laptop
    UDP, one JSON object per datagram, sent to the IP address the latest
    valid command packet came from, port TELEM_PORT.
    {"depth_m": 1.23, "pressure_kpa": 113.6, "water_c": 8.70, "internal_c": 31.2,
     "roll": 1.20, "pitch": -0.40, "yaw": 214.0, "voltage": 15.80,
     "leak": false, "faults": ["TEMP"], "fault_flags": 8, "stm32": true,
     "uart_errors": 0, "ack": 1234, "rx_cmds": 5678}
    SI units throughout: m, kPa, degC, deg, V.
    A sensor whose fault bit is set (e.g. FAULT_DEPTH) has its value(s) sent
    as null, because the STM32 is holding a stale reading. While the STM32 is
    not answering at all, "stm32" is false and every sensor value is null; the
    Pi still sends ~10 Hz heartbeats (with ack / rx_cmds) so the command centre
    can see that the tether and the Pi itself are fine.

FAILSAFE
    No valid command packet for CMD_TIMEOUT -> thrusters 0 % (tilt held).
    The bridge keeps sending frames to the STM32 at 50 Hz regardless, so the
    STM32's own UART timeout only trips if the Pi program itself dies.
    On exit (Ctrl+C or systemctl stop) a stop frame is sent.

-----------------------------------------------------------------------------
Run (needs chmod +x once):
    ./rov_bridge.py
Options (all optional):
    --serial /dev/serial0  --cmd-port 5600  --telem-port 5601  --verbose
"""

import argparse
import binascii
import json
import signal
import socket
import struct
import threading
import time

import serial

from UART_DMA import (build_cmd_frame, TelemetryReader,
                      FAULT_IMU, FAULT_DEPTH, FAULT_TEMP)

# ---------------- settings ----------------
SERIAL_PORT = "/dev/serial0"   # PL011 UART: dtoverlay=disable-bt in config.txt
BAUD = 115200
CMD_PORT = 5600                # laptop -> Pi (must match the command centre profile)
TELEM_PORT = 5601              # Pi -> laptop

UART_PERIOD = 1 / 50           # s   -> command frames to the STM32 at 50 Hz
CMD_TIMEOUT = 0.25             # s   without a valid command packet -> zero thrusters
STM32_TIMEOUT = 0.5            # s   without valid telemetry -> STM32 treated as not answering
HEARTBEAT_PERIOD = 0.1         # s   telemetry rate while the STM32 is not answering
STATUS_PERIOD = 5.0            # s   between status lines printed to the terminal

# ---------------- command packet (must match command_center_py/link.py) ----------------
CMD_STRUCT = struct.Struct("<2sBBH6bbx")      # 14 bytes, then a 2-byte CRC
CMD_PACKET_SIZE = CMD_STRUCT.size + 2         # 16
CMD_MAGIC = b"RV"
CMD_VERSION = 1
FLAG_ARMED = 1 << 0
FLAG_RECOVER = 1 << 1

ZERO_MOTORS = [0, 0, 0, 0, 0, 0]

# Which telemetry keys go stale when a given STM32 fault bit is set.
STALE_KEYS = {
    FAULT_DEPTH: ("depth_m", "pressure_kpa"),
    FAULT_TEMP: ("water_c",),
    FAULT_IMU: ("roll", "pitch", "yaw"),
}
SENSOR_KEYS = ("depth_m", "pressure_kpa", "water_c", "internal_c",
               "roll", "pitch", "yaw", "voltage", "leak")


def parse_command(pkt: bytes):
    """Returns (seq, armed, recover, motors, tilt_deg) for a valid packet, else None."""
    if len(pkt) != CMD_PACKET_SIZE:
        return None
    body, (crc,) = pkt[:CMD_STRUCT.size], struct.unpack("<H", pkt[CMD_STRUCT.size:])
    if binascii.crc_hqx(body, 0xFFFF) != crc:
        return None
    magic, version, flags, seq, *rest = CMD_STRUCT.unpack(body)
    if magic != CMD_MAGIC or version != CMD_VERSION:
        return None
    motors, tilt = list(rest[:6]), rest[6]
    return seq, bool(flags & FLAG_ARMED), bool(flags & FLAG_RECOVER), motors, tilt


def telem_to_json_dict(t):
    """Maps a UART_DMA telemetry dict to the command centre's JSON keys (SI units)."""
    out = {
        "depth_m": round(t["depth_m"], 2),
        "pressure_kpa": round(t["pressure_kpa"], 1),
        "water_c": round(t["water_temp_c"], 2),
        "internal_c": round(t["inside_temp_c"], 2),
        "roll": round(t["roll_deg"], 2),
        "pitch": round(t["pitch_deg"], 2),
        "yaw": round(t["heading_deg"], 2),
        "voltage": round(t["battery_v"], 2),
        "leak": t["leak"],
        "faults": t["faults"],
        "fault_flags": t["fault_flags"],
    }
    for bit, keys in STALE_KEYS.items():
        if t["fault_flags"] & bit:
            for k in keys:
                out[k] = None
    return out


class Bridge:
    def __init__(self, args):
        self.verbose = args.verbose
        self.telem_port = args.telem_port
        self.stop = threading.Event()
        self.lock = threading.Lock()

        # latest command from the laptop
        self.laptop_ip = None
        self.last_cmd_time = float("-inf")
        self.last_seq = None
        self.rx_cmds = 0
        self.bad_cmds = 0
        self.armed = False
        self.recover = False
        self.motors = list(ZERO_MOTORS)
        self.tilt_deg = 0.0

        # latest telemetry from the STM32
        self.last_stm_time = float("-inf")
        self.stm_frames = 0

        self.ser = serial.Serial(port=args.serial, baudrate=BAUD,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, timeout=0.1)
        self.reader = TelemetryReader()

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd_sock.bind(("0.0.0.0", args.cmd_port))
        self.cmd_sock.settimeout(0.1)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"ROV bridge: UDP {args.cmd_port} in / {args.telem_port} out  <->  "
              f"{args.serial} @ {BAUD} baud", flush=True)

    # ---------------- laptop -> Pi ----------------
    def udp_rx_loop(self):
        while not self.stop.is_set():
            try:
                pkt, (ip, _port) = self.cmd_sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                if self.stop.is_set():
                    return
                raise
            cmd = parse_command(pkt)
            if cmd is None:
                self.bad_cmds += 1
                continue
            seq, armed, recover, motors, tilt = cmd
            with self.lock:
                if ip != self.laptop_ip:
                    print(f"*** Command centre at {ip} -- sending telemetry to "
                          f"{ip}:{self.telem_port} ***", flush=True)
                    self.laptop_ip = ip
                if armed != self.armed:
                    print(f"*** {'ARMED' if armed else 'DISARMED'} by command centre ***", flush=True)
                self.last_cmd_time = time.monotonic()
                self.last_seq = seq
                self.rx_cmds += 1
                self.armed = armed
                self.recover = recover
                self.motors = motors
                self.tilt_deg = float(tilt)

    # ---------------- Pi -> STM32 ----------------
    def uart_tx_loop(self):
        cmd_was_live = False
        next_t = time.monotonic()
        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                cmd_live = now - self.last_cmd_time < CMD_TIMEOUT
                motors = self.motors if (cmd_live and self.armed) else ZERO_MOTORS
                tilt, recover = self.tilt_deg, (self.recover and cmd_live)

            if cmd_was_live and not cmd_live:
                print(f"*** No commands for {CMD_TIMEOUT} s -- thrusters zeroed ***", flush=True)
            elif cmd_live and not cmd_was_live:
                print("*** Receiving commands from the command centre ***", flush=True)
            cmd_was_live = cmd_live

            # One complete frame per write() -- the STM32 finds frames by idle line.
            self.ser.write(build_cmd_frame(motors=motors, cam_tilt_deg=tilt, recover=recover))

            next_t += UART_PERIOD
            delay = next_t - time.monotonic()
            if delay < -UART_PERIOD:          # fell far behind (e.g. system stall): resync
                next_t = time.monotonic()
                delay = 0.0
            time.sleep(max(0.0, delay))

    # ---------------- STM32 -> Pi -> laptop ----------------
    def uart_rx_loop(self):
        last_heartbeat = 0.0
        stm_was_up = False
        while not self.stop.is_set():
            data = self.ser.read(64)          # b"" after the 0.1 s timeout
            now = time.monotonic()
            frames = list(self.reader.feed(data)) if data else []

            for t in frames:
                self.last_stm_time = now
                self.stm_frames += 1
                msg = telem_to_json_dict(t)
                msg["stm32"] = True
                self.send_telem(msg)

            stm_up = now - self.last_stm_time < STM32_TIMEOUT
            if stm_up != stm_was_up:
                print("*** STM32 telemetry " + ("received -- link up" if stm_up
                      else f"lost -- nothing for {STM32_TIMEOUT} s") + " ***", flush=True)
                stm_was_up = stm_up

            # Heartbeat while the STM32 is silent, so the laptop still sees the Pi.
            if not stm_up and now - last_heartbeat >= HEARTBEAT_PERIOD:
                last_heartbeat = now
                msg = {k: None for k in SENSOR_KEYS}
                msg.update({"faults": [], "fault_flags": None, "stm32": False})
                self.send_telem(msg)

    def send_telem(self, msg):
        with self.lock:
            ip = self.laptop_ip
            msg["ack"] = self.last_seq
            msg["rx_cmds"] = self.rx_cmds
        if ip is None:
            return                            # no command centre has talked to us yet
        msg["uart_errors"] = self.reader.checksum_errors
        try:
            self.telem_sock.sendto(json.dumps(msg, separators=(",", ":")).encode(),
                                   (ip, self.telem_port))
        except OSError as exc:                # e.g. laptop unplugged: keep going
            if self.verbose:
                print(f"telemetry send failed: {exc}", flush=True)

    # ---------------- status ----------------
    def status_loop(self):
        while not self.stop.wait(STATUS_PERIOD):
            now = time.monotonic()
            with self.lock:
                cmd_age = now - self.last_cmd_time
                line = (f"[status] laptop={self.laptop_ip or '-'}  "
                        f"cmds={self.rx_cmds} (bad {self.bad_cmds})  "
                        f"last cmd {'%.2f s ago' % cmd_age if cmd_age < 1e9 else 'never'}  "
                        f"armed={self.armed}  motors={self.motors if self.armed else ZERO_MOTORS}  "
                        f"tilt={self.tilt_deg:+.0f} deg  ")
            line += (f"stm32 frames={self.stm_frames} "
                     f"({'up' if now - self.last_stm_time < STM32_TIMEOUT else 'DOWN'}, "
                     f"checksum errors {self.reader.checksum_errors})")
            print(line, flush=True)

    # ---------------- run / shutdown ----------------
    def run(self):
        threads = [threading.Thread(target=f, daemon=True, name=f.__name__)
                   for f in (self.udp_rx_loop, self.uart_rx_loop, self.status_loop)]
        for th in threads:
            th.start()
        try:
            self.uart_tx_loop()               # main thread: fixed-rate UART output
        finally:
            self.shutdown()

    def shutdown(self):
        print("\nStopping -- sending stop frame to STM32", flush=True)
        self.stop.set()
        try:
            self.ser.write(build_cmd_frame(motors=ZERO_MOTORS, cam_tilt_deg=0.0))
            self.ser.flush()
        finally:
            self.ser.close()
            self.cmd_sock.close()
            self.telem_sock.close()


def main():
    ap = argparse.ArgumentParser(description="Command centre <-> STM32 bridge")
    ap.add_argument("--serial", default=SERIAL_PORT)
    ap.add_argument("--cmd-port", type=int, default=CMD_PORT)
    ap.add_argument("--telem-port", type=int, default=TELEM_PORT)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    # systemctl stop sends SIGTERM: turn it into the same clean exit as Ctrl+C.
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    bridge = Bridge(args)
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
