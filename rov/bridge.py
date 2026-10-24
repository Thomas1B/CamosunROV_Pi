"""
Bridge: command centre (Ethernet/UDP) <-> Raspberry Pi <-> STM32 (UART).

    laptop --UDP 5600, 16-byte command packet, 50 Hz--> Pi --UART 11-byte frame, 50 Hz--> STM32
    laptop --UDP 5600, 16-byte reference packet-------> Pi --UART 10-byte frame, on change> STM32
    laptop <--UDP 5601, JSON telemetry, 50 Hz---------- Pi <--UART 20-byte frame, 50 Hz-- STM32

Started by main.py in its own process. The UART side reuses rov/UART_DMA.py
unchanged (the same code Test_UART_DMA.py uses). Only one program can use
/dev/serial0 at a time, so stop main.py before running Test_UART_DMA.py.

Console output (same style as Test_UART_DMA.py), every line tagged [BRIDGE]:
    - "Waiting for ..." every WAIT_PRINT_PERIOD while the STM32 / command centre
      is not connected, one message when each connects, one if it is lost
    - telemetry + command lines at PRINT_PERIOD (2 Hz)
    - leak / fault changes and ARM / DISARM straight away

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

REFERENCE PACKET  laptop -> Pi  (matches command_center_py/link.py)
    Bar30 depth-calculation settings, set in the command centre's CONNECT dialog or
    File > Reference Values... Same port as the command packet, told apart by its magic.
    UDP, CMD_PORT, 16 bytes, little-endian, struct "<2sBBHffH"
    0  2s  magic     b"RF"
    2  B   version   1
    3  B   flags     bit0 = second-order pressure calibration on (water below 20 C)
    4  H   ref_seq   +1 each time the values change in the command centre
    6  f   surface   surface pressure reference, hPa (= mbar), float32
    10 f   density   water density, kg/m^3, float32
    14 H   crc16     CRC-16/CCITT-FALSE over bytes 0..13
    Dropped if malformed or out of range (surface 800..1100 hPa, density 900..1100 kg/m^3).
    A new ref_seq is forwarded to the STM32 as a 10-byte reference frame (see
    rov/UART_DMA.py), REF_UART_REPEATS times, and re-sent whenever the STM32 reconnects.
    Once forwarded, telemetry carries "ref_ack": ref_seq. The command centre resends
    the packet every second until it sees that ack (e.g. while the STM32 is not
    connected, or after the Pi restarts and ref_ack is null again).

TELEMETRY  Pi -> laptop
    UDP, one JSON object per datagram, sent to the IP address the latest
    valid command packet came from, port TELEM_PORT.
    {"depth_m": 1.23, "pressure_kpa": 113.6, "water_c": 8.70, "internal_c": 31.2,
     "roll": 1.20, "pitch": -0.40, "yaw": 214.0, "voltage": 15.80,
     "leak": false, "faults": ["TEMP"], "fault_flags": 8, "stm32": true,
     "uart_errors": 0, "ack": 1234, "rx_cmds": 5678, "ref_ack": 3}
    SI units throughout: m, kPa, degC, deg, V.
    ref_ack is the ref_seq of the reference values last forwarded to the STM32
    (null until the command centre has sent any).
    A sensor whose fault bit is set (e.g. FAULT_DEPTH) has its value(s) sent
    as null, because the STM32 is holding a stale reading. While the STM32 is
    not answering at all, "stm32" is false and every sensor value is null; the
    Pi still sends ~10 Hz heartbeats (with ack / rx_cmds) so the command centre
    can see that the tether and the Pi itself are fine.

FAILSAFE
    No valid command packet for CMD_TIMEOUT -> thrusters 0 % (tilt held).
    Frames keep going to the STM32 at 50 Hz regardless, so the STM32's own
    UART timeout only trips if this program itself dies.
    On exit (Ctrl+C or systemctl stop) a stop frame is sent.
"""

import binascii
import errno
import json
import signal
import sys
import socket
import struct
import threading
import time

import serial

from rov.UART_DMA import (build_cmd_frame, build_ref_frame, TelemetryReader,
                          FAULT_LEAK, FAULT_IMU, FAULT_DEPTH, FAULT_TEMP)
from rov import settings as cfg

# ---------------- command packet (must match command_center_py/link.py) ----------------
CMD_STRUCT = struct.Struct("<2sBBH6bbx")      # 14 bytes, then a 2-byte CRC
CMD_PACKET_SIZE = CMD_STRUCT.size + 2         # 16
CMD_MAGIC = b"RV"
CMD_VERSION = 1
FLAG_ARMED = 1 << 0
FLAG_RECOVER = 1 << 1

# ---------------- reference packet (must match command_center_py/link.py) ----------------
REF_STRUCT = struct.Struct("<2sBBHff")        # 14 bytes, then a 2-byte CRC
REF_PACKET_SIZE = REF_STRUCT.size + 2         # 16
REF_MAGIC = b"RF"
REF_VERSION = 1
REF_FLAG_SECOND_ORDER = 1 << 0
SURFACE_RANGE_HPA = (800.0, 1100.0)           # sanity limits, same as the command centre
DENSITY_RANGE = (900.0, 1100.0)

ZERO_MOTORS = [0, 0, 0, 0, 0, 0]
STOP_FRAMES = 3                               # stop frames sent on exit, UART_PERIOD apart

# Which telemetry keys go stale when a given STM32 fault bit is set.
STALE_KEYS = {
    FAULT_DEPTH: ("depth_m", "pressure_kpa"),
    FAULT_TEMP: ("water_c",),
    FAULT_IMU: ("roll", "pitch", "yaw"),
}
SENSOR_KEYS = ("depth_m", "pressure_kpa", "water_c", "internal_c",
               "roll", "pitch", "yaw", "voltage", "leak")


def say(msg):
    # One write per line, so lines from the bridge, camera and main processes
    # never get mixed together on the shared console.
    sys.stdout.write(f"[BRIDGE] {msg}\n")
    sys.stdout.flush()


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


def parse_reference(pkt: bytes):
    """Returns (ref_seq, surface_hpa, density, second_order) for a valid packet, else None."""
    if len(pkt) != REF_PACKET_SIZE:
        return None
    body, (crc,) = pkt[:REF_STRUCT.size], struct.unpack("<H", pkt[REF_STRUCT.size:])
    if binascii.crc_hqx(body, 0xFFFF) != crc:
        return None
    magic, version, flags, ref_seq, surface, density = REF_STRUCT.unpack(body)
    if magic != REF_MAGIC or version != REF_VERSION:
        return None
    if not (SURFACE_RANGE_HPA[0] <= surface <= SURFACE_RANGE_HPA[1]
            and DENSITY_RANGE[0] <= density <= DENSITY_RANGE[1]):
        return None
    return ref_seq, round(surface, 2), round(density, 1), bool(flags & REF_FLAG_SECOND_ORDER)


def describe_refs(refs):
    surface, density, second = refs
    return f"{surface:.2f} hPa, {density:.1f} kg/m^3, second-order {'ON' if second else 'OFF'}"


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
    def __init__(self):
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

        # Bar30 reference values from the laptop, forwarded to the STM32
        self.refs = None              # (surface_hpa, density, second_order), None until received
        self.ref_seq = None           # laptop's ref_seq for self.refs
        self.ref_id = 0               # UART ref_id, +1 per new set of values
        self.ref_pending = 0          # reference frames still to send to the STM32
        self.ref_acked = None         # ref_seq last forwarded -> telemetry "ref_ack"

        # latest telemetry from the STM32
        self.last_stm_time = float("-inf")
        self.stm_frames = 0
        self.latest_telem = None

        self.ser = serial.Serial(port=cfg.SERIAL_PORT, baudrate=cfg.BAUD,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, timeout=0.1)
        self.reader = TelemetryReader()

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd_sock.bind(("0.0.0.0", cfg.CMD_PORT))
        self.cmd_sock.settimeout(0.1)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        say(f"Started: UDP {cfg.CMD_PORT} in / {cfg.TELEM_PORT} out  <->  "
            f"{cfg.SERIAL_PORT} @ {cfg.BAUD} baud")

    def stm_up(self, now):
        return now - self.last_stm_time < cfg.STM32_TIMEOUT

    def cmd_up(self, now):
        return now - self.last_cmd_time < cfg.CMD_TIMEOUT

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
            if pkt[:2] == REF_MAGIC:
                self.handle_reference(pkt)
                continue
            cmd = parse_command(pkt)
            if cmd is None:
                self.bad_cmds += 1
                continue
            seq, armed, recover, motors, tilt = cmd
            with self.lock:
                self.laptop_ip = ip
                if armed != self.armed:
                    say(f">>> {'ARMED' if armed else 'DISARMED'} by command centre")
                if recover and not self.recover:
                    say(">>> Recover command from command centre (clears a leak shutdown if dry)")
                self.last_cmd_time = time.monotonic()
                self.last_seq = seq
                self.rx_cmds += 1
                self.armed = armed
                self.recover = recover
                self.motors = motors
                self.tilt_deg = float(tilt)

    def handle_reference(self, pkt):
        ref = parse_reference(pkt)
        if ref is None:
            self.bad_cmds += 1
            return
        ref_seq, *values = ref
        values = tuple(values)
        with self.lock:
            if ref_seq == self.ref_seq and values == self.refs:
                return                        # a resend of what we already have
            self.refs, self.ref_seq = values, ref_seq
            self.ref_id = (self.ref_id + 1) & 0xFF
            self.ref_pending = cfg.REF_UART_REPEATS
            stm = self.stm_up(time.monotonic())
        say(f">>> Reference values from command centre (#{ref_seq}): {describe_refs(values)}"
            + ("" if stm else " -- will forward when the STM32 connects"))

    def next_ref_frame(self, now):
        """The reference frame to send this UART period, or None."""
        with self.lock:
            if self.ref_pending <= 0 or self.refs is None or not self.stm_up(now):
                return None
            self.ref_pending -= 1
            frame = build_ref_frame(*self.refs, self.ref_id)
            done = self.ref_pending == 0
            if done:
                self.ref_acked = self.ref_seq
            refs, seq = self.refs, self.ref_seq
        if done:
            say(f"*** Reference values #{seq} sent to STM32: {describe_refs(refs)} ***")
        return frame

    # ---------------- Pi -> STM32 ----------------
    def uart_tx_loop(self):
        connected = False
        last_cmd_seen = time.monotonic()
        waiting_since = last_cmd_seen
        last_wait_print = float("-inf")
        next_t = time.monotonic()

        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                cmd_live = self.cmd_up(now)
                motors = self.motors if (cmd_live and self.armed) else ZERO_MOTORS
                tilt, recover = self.tilt_deg, (self.recover and cmd_live)
                ip = self.laptop_ip

            # command centre connection messages
            if cmd_live:
                last_cmd_seen = now
                if not connected:
                    connected = True
                    say(f"*** Connected to command centre at {ip} -- commands received "
                        f"(waited {now - waiting_since:.1f} s), telemetry going to "
                        f"{ip}:{cfg.TELEM_PORT} ***")
            else:
                if connected:
                    connected = False
                    waiting_since = last_cmd_seen
                    last_wait_print = now
                    say(f"*** Command centre lost -- no commands for {cfg.CMD_TIMEOUT} s, "
                        f"thrusters set to 0 % ***")
                if now - last_wait_print >= cfg.WAIT_PRINT_PERIOD:
                    last_wait_print = now
                    say(f"Waiting for command centre... ({now - waiting_since:.0f} s)")

            # One complete frame per write() -- the STM32 finds frames by idle line.
            self.ser.write(build_cmd_frame(motors=motors, cam_tilt_deg=tilt, recover=recover))

            # New Bar30 reference values: their own frame, half a period later, so there
            # is an idle gap on both sides and the 50 Hz command frames are not disturbed.
            ref_frame = self.next_ref_frame(now)
            if ref_frame is not None:
                time.sleep(cfg.UART_PERIOD / 2)
                self.ser.write(ref_frame)

            next_t += cfg.UART_PERIOD
            delay = next_t - time.monotonic()
            if delay < -cfg.UART_PERIOD:      # fell far behind (e.g. system stall): resync
                next_t = time.monotonic()
                delay = 0.0
            time.sleep(max(0.0, delay))

    # ---------------- STM32 -> Pi -> laptop ----------------
    def uart_rx_loop(self):
        connected = False
        last_rx = time.monotonic()
        waiting_since = last_rx
        last_wait_print = float("-inf")
        last_heartbeat = 0.0
        prev_telem = None

        while not self.stop.is_set():
            data = self.ser.read(64)          # b"" after the 0.1 s timeout
            # Only VALID frames (START + checksum ok) count as a connection --
            # random noise on an unplugged wire is rejected by the reader.
            frames = list(self.reader.feed(data)) if data else []
            now = time.monotonic()

            if frames:
                last_rx = now
                if not connected:
                    connected = True
                    say(f"*** Connected to STM32 -- telemetry received "
                        f"(waited {now - waiting_since:.1f} s) ***")
                    with self.lock:           # it may have restarted: give it the reference values again
                        if self.refs is not None:
                            self.ref_pending = cfg.REF_UART_REPEATS
                for t in frames:
                    self.stm_frames += 1
                    report_state_change(prev_telem, t)     # leak / faults: right away
                    prev_telem = t
                    with self.lock:
                        self.last_stm_time = now
                        self.latest_telem = t
                    msg = telem_to_json_dict(t)
                    msg["stm32"] = True
                    self.send_telem(msg)

            elif now - last_rx > cfg.STM32_TIMEOUT:
                if connected:
                    connected = False
                    prev_telem = None
                    waiting_since = last_rx
                    last_wait_print = now
                    say(f"*** STM32 connection lost -- no telemetry for {cfg.STM32_TIMEOUT} s ***")
                if now - last_wait_print >= cfg.WAIT_PRINT_PERIOD:
                    last_wait_print = now
                    say(f"Waiting for connection to STM32... ({now - waiting_since:.0f} s)")

                # Heartbeat while the STM32 is silent, so the laptop still sees the Pi.
                if now - last_heartbeat >= cfg.HEARTBEAT_PERIOD:
                    last_heartbeat = now
                    msg = {k: None for k in SENSOR_KEYS}
                    msg.update({"faults": [], "fault_flags": None, "stm32": False})
                    self.send_telem(msg)

    def send_telem(self, msg):
        with self.lock:
            ip = self.laptop_ip
            msg["ack"] = self.last_seq
            msg["rx_cmds"] = self.rx_cmds
            msg["ref_ack"] = self.ref_acked
        if ip is None:
            return                            # no command centre has talked to us yet
        msg["uart_errors"] = self.reader.checksum_errors
        try:
            self.telem_sock.sendto(json.dumps(msg, separators=(",", ":")).encode(),
                                   (ip, cfg.TELEM_PORT))
        except OSError:
            pass                              # e.g. tether unplugged: keep going

    # ---------------- 2 Hz console lines ----------------
    def print_loop(self):
        while not self.stop.wait(cfg.PRINT_PERIOD):
            now = time.monotonic()
            with self.lock:
                telem = self.latest_telem if self.stm_up(now) else None
                cmd_live = self.cmd_up(now)
                armed, motors, tilt = self.armed, list(self.motors), self.tilt_deg
                rx_cmds, bad = self.rx_cmds, self.bad_cmds
            if telem is not None:
                print_telemetry(self.stm_frames, telem, self.reader.checksum_errors)
            if cmd_live:
                sent = motors if armed else ZERO_MOTORS
                say(f"         Command: {'ARMED   ' if armed else 'DISARMED'}  "
                    f"Motors: {sent} %  Tilt: {tilt:+.0f} deg  "
                    f"(packets {rx_cmds}, bad {bad})")

    # ---------------- run / shutdown ----------------
    def run(self):
        threads = [threading.Thread(target=f, daemon=True, name=f.__name__)
                   for f in (self.udp_rx_loop, self.uart_rx_loop, self.print_loop)]
        for th in threads:
            th.start()
        try:
            self.uart_tx_loop()               # this thread: fixed-rate UART output
        finally:
            self.shutdown()

    def shutdown(self):
        # Ctrl+C reaches this process AND main.py, which then also sends SIGTERM:
        # ignore both from here on so nothing interrupts the stop frames below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        say("Stopping -- sending stop frame to STM32 (thrusters 0 %, camera level)")
        self.stop.set()
        try:
            # Ctrl+C can land in the middle of a frame being written. The STM32 finds
            # frames by the idle gap between them, so leave a gap first, then send the
            # stop frame a few times so at least one arrives cleanly.
            stop_frame = build_cmd_frame(motors=ZERO_MOTORS, cam_tilt_deg=0.0)
            for _ in range(STOP_FRAMES):
                time.sleep(cfg.UART_PERIOD)
                self.ser.write(stop_frame)
                self.ser.flush()   # make sure it actually leaves before closing
        finally:
            self.ser.close()
            self.cmd_sock.close()
            self.telem_sock.close()


def print_telemetry(frames, telem, checksum_errors):
    faults = ", ".join(telem["faults"]) or "none"
    say(
        f"[{frames:6d}] "
        f"Pressure: {telem['pressure_kpa']:.1f} kPa  "
        f"Depth: {telem['depth_m']:.2f} m  "
        f"Water: {telem['water_temp_c']:.2f} C  "
        f"Battery: {telem['battery_v']:.2f} V  "
        f"Inside: {telem['inside_temp_c']:.2f} C  "
        f"Heading: {telem['heading_deg']:.2f} deg  "
        f"Roll: {telem['roll_deg']:.2f} deg  "
        f"Pitch: {telem['pitch_deg']:.2f} deg  "
        f"Leak: {'YES' if telem['leak'] else 'no'}  "
        f"Faults: {faults}  "
        f"Lost (checksum): {checksum_errors}"
    )


def report_state_change(prev, telem):
    """Prints leak / fault changes as soon as they happen."""
    if prev is None:
        return
    if telem["leak"] != prev["leak"]:
        if telem["leak"]:
            say("!!! LEAK SENSOR WET -- STM32 has shut down thrusters and tilt servo !!!")
        else:
            say("*** Leak sensor dry again -- a recover command clears the shutdown ***")

    was_shutdown = bool(prev["fault_flags"] & FAULT_LEAK)
    is_shutdown = bool(telem["fault_flags"] & FAULT_LEAK)
    if was_shutdown and not is_shutdown:
        say("*** Leak shutdown cleared -- STM32 back to normal operation ***")

    if telem["fault_flags"] != prev["fault_flags"]:
        faults = ", ".join(telem["faults"]) or "none"
        say(f"*** Faults changed: {faults} ***")


def run():
    """Process entry point used by main.py."""
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        bridge = Bridge()
    except KeyboardInterrupt:
        return
    except serial.SerialException as exc:
        say(f"!!! Cannot open {cfg.SERIAL_PORT}: {exc}")
        say("    Check: UART enabled in config.txt, user in the 'dialout' group, "
            "and Test_UART_DMA.py not running")
        raise SystemExit(1)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            say(f"!!! UDP port {cfg.CMD_PORT} is already in use -- is another copy running? "
                f"(old rov_bridge.py, a second main.py, or the rov service: "
                f"sudo systemctl stop rov)")
            raise SystemExit(1)
        raise
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
