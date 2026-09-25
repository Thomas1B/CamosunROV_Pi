"""
Fixed-length UART protocol, matching UART_DMA.h/.c on the STM32 side.

Command frame (Pi -> STM32), 10 bytes:
    START | motor1..motor6 (i8 each) | cam_tilt_dc (i16) | CHECKSUM

Telemetry frame (STM32 -> Pi), 18 bytes:
    START | depth_dm (i16) | water_temp_dc (i16) | battery_dv (u16) | inside_temp_dc (i16)
          | heading_dc (u16) | roll_dc (i16) | pitch_dc (i16)
          | leak (u8) | fault_flags (u8) | CHECKSUM

cam_tilt_dc: camera tilt x10 (deci-degrees), 0 = level, + = up.
No clamping on the Pi -- values are sent as given and the STM32's motor/tilt
control functions limit them. Values must still fit their wire field
(motors -128..127, tilt -3276.8..3276.7 deg), otherwise struct.pack raises
struct.error instead of sending a frame.
leak:        live leak-sensor state, 0 = dry, 1 = water detected right now.
fault_flags: the STM32's latched SystemState_t bit flags (see FAULT_* below).
    `leak` says whether there is water NOW; FAULT_LEAK says there EVER was.

heading/roll/pitch come from the BNO055 IMU, scaled x10 (deci-degrees) like
everything else in this protocol.

All multi-byte values are little-endian (native STM32/Cortex-M4 byte order).
All sensor/command values are integers scaled x10 ("deci-units"); divide by
10.0 to get real units (e.g. depth_dm=2995 -> 299.5 m).

CHECKSUM is a simple additive checksum (sum of all body bytes, mod 256) --
not a CRC. Good enough for a short, physically wired UART link where bit
errors are rare; it catches dropped/corrupted bytes but not e.g. two bytes
swapping places, since addition doesn't care about order.

The STM32 receives with idle-line DMA: it finds frame boundaries from
the silence between frames, so always send each command frame in a single
ser.write() call.

Keep this file in sync with UART_DMA.h/.c on the STM32 side.
"""

import struct

START_BYTE = 0xAA

# struct format strings (excluding START and CHECKSUM, which we handle separately)
_CMD_BODY_FMT = "<6bh"           # 6x motor(i8), cam_tilt_dc(i16)
_TELEM_BODY_FMT = "<hhHhHhhBB"   # depth_dm(u16), water_temp_dc(i16), battery_dv(u16),
                                 # inside_temp_dc(i16), heading_dc(u16), roll_dc(i16),
                                 # pitch_dc(i16), leak(u8), fault_flags(u8)

CMD_FRAME_SIZE = 1 + struct.calcsize(_CMD_BODY_FMT) + 1      # 10
TELEM_FRAME_SIZE = 1 + struct.calcsize(_TELEM_BODY_FMT) + 1  # 18

# Fault bit flags -- must match SystemState_t on the STM32.
FAULT_NONE = 0
FAULT_LEAK = 1 << 0    # leak detected
FAULT_IMU = 1 << 1     # IMU failure or invalid data
FAULT_DEPTH = 1 << 2   # depth sensor failure or out-of-range reading
FAULT_TEMP = 1 << 3    # temperature sensor failure or out-of-range reading
FAULT_UART = 1 << 4    # UART communication failure

FAULT_NAMES = {
    FAULT_LEAK: "LEAK",
    FAULT_IMU: "IMU",
    FAULT_DEPTH: "DEPTH",
    FAULT_TEMP: "TEMP",
    FAULT_UART: "UART",
}


def checksum8(data: bytes) -> int:
    """Simple additive checksum: sum of all bytes, wrapped to 8 bits."""
    return sum(data) & 0xFF


def decode_faults(flags: int) -> list[str]:
    """Turns a fault_flags byte into a list of names, e.g. 0x09 -> ['LEAK', 'TEMP']."""
    return [name for bit, name in FAULT_NAMES.items() if flags & bit]


def build_cmd_frame(motors: list[int], cam_tilt_deg: float = 0.0) -> bytes:
    """
    motors:       list of 6 values, -100..100 (%) expected
    cam_tilt_deg: camera tilt in degrees, -90..90 expected, 0 = level, + = up
    Not clamped here -- the STM32 limits them.
    """
    if len(motors) != 6:
        raise ValueError("expected exactly 6 motor values")
    motors = [int(round(m)) for m in motors]
    cam_tilt_dc = int(round(cam_tilt_deg * 10))

    body = struct.pack(_CMD_BODY_FMT, *motors, cam_tilt_dc)
    frame_checksum = checksum8(body)
    return bytes([START_BYTE]) + body + bytes([frame_checksum])


def parse_telem_frame(frame: bytes):
    """
    Parses a full TELEM_FRAME_SIZE-byte frame (including START and CHECKSUM).
    Returns a dict of real-unit values, or None if START/CHECKSUM don't check out.
    """
    if len(frame) != TELEM_FRAME_SIZE:
        return None
    if frame[0] != START_BYTE:
        return None

    body = frame[1:-1]
    received_checksum = frame[-1]
    if checksum8(body) != received_checksum:
        return None

    (depth_dm, water_temp_dc, battery_dv, inside_temp_dc,
     heading_dc, roll_dc, pitch_dc, leak, fault_flags) = struct.unpack(_TELEM_BODY_FMT, body)

    return {
        "depth_m": depth_dm / 10.0,
        "water_temp_c": water_temp_dc / 10.0,
        "battery_v": battery_dv / 10.0,
        "inside_temp_c": inside_temp_dc / 10.0,
        "heading_deg": heading_dc / 10.0,
        "roll_deg": roll_dc / 10.0,
        "pitch_deg": pitch_dc / 10.0,
        "leak": bool(leak),
        "fault_flags": fault_flags,
        "faults": decode_faults(fault_flags),
    }


class TelemetryReader:
    """
    Byte-hunting reader for fixed-length telemetry frames coming from the
    STM32: scan for START, then check exactly the fixed number of bytes.

    Usage: call feed(data) with whatever bytes arrived from serial; it
    yields a parsed telemetry dict for each complete, checksum-valid frame
    found.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)

        while True:
            # Find the next START byte, discarding anything before it.
            start_idx = self._buf.find(START_BYTE)
            if start_idx == -1:
                self._buf.clear()
                return
            if start_idx > 0:
                del self._buf[:start_idx]

            # Do we have a full frame yet?
            if len(self._buf) < TELEM_FRAME_SIZE:
                return

            candidate = bytes(self._buf[:TELEM_FRAME_SIZE])
            parsed = parse_telem_frame(candidate)

            # Whether it parsed or not, consume the START byte we just
            # examined so we don't loop on it forever, then keep scanning.
            del self._buf[0:1]

            if parsed is not None:
                # also consume the rest of the frame we just used
                del self._buf[: TELEM_FRAME_SIZE - 1]
                yield parsed