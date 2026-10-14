#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3
"""
Test script for UART_DMA: sends command frames at 50 Hz and prints
telemetry coming back from the STM32.

While no valid telemetry is arriving (STM32 off, booting, or unplugged), it
prints a "waiting for connection" message every WAIT_PRINT_PERIOD seconds.
It reports once when the connection comes up, and once if it is lost.

Leak / fault changes are reported immediately (not held back by the 2 Hz
print rate).

Keyboard (type the letter, then press Enter):
    r  -> send a recover command to clear a leak shutdown. The STM32 only
          accepts it if its leak sensor is dry; if still wet it is ignored
          and FAULT_LEAK stays set.
    q  -> quit (same as Ctrl+C)

Run it directly (uses the venv via the shebang; needs chmod +x once):
    ./Test_UART_DMA.py
or with the venv's interpreter explicitly:
    ~/CamosunROV_Pi/ROV_venv/bin/python3 Test_UART_DMA.py
or activate the venv first and run:
    source ~/CamosunROV_Pi/ROV_venv/bin/activate
    python3 Test_UART_DMA.py
"""

import serial
import time
import threading

from UART_DMA import build_cmd_frame, TelemetryReader, FAULT_LEAK

CMD_PERIOD = 1/50        # s -> 50 Hz command rate
PRINT_PERIOD = 1/2       # s -> print telemetry at 2 Hz so the terminal stays readable

LINK_TIMEOUT = 0.5       # s without a valid telemetry frame -> treated as not connected
                         # (telemetry normally arrives every 0.02 s, so 0.5 s = 25 missed frames)
WAIT_PRINT_PERIOD = 2.0  # s between "waiting for connection" messages

RECOVER_FRAMES = 5       # frames sent with recover=1 per request (5 x 0.02 s = 0.1 s),
                         # so one dropped frame doesn't lose the request. The STM32
                         # acts on the 0 -> 1 edge only, so holding 1 briefly is safe.

# Test command sent every frame
TEST_MOTORS = [50, 0, -100, 30, 0, 0]   # %
TEST_TILT_DEG = -15.0                   # deg

ser = serial.Serial(
    port='/dev/serial0',   # use the PL011 UART: dtoverlay=disable-bt in config.txt
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1,
)

reader = TelemetryReader()
stop_event = threading.Event()

# Set by the keyboard thread, consumed by the TX loop.
recover_request = threading.Event()


def print_telemetry(frames, telem):
    faults = ", ".join(telem["faults"]) or "none"
    print(
        f"[{frames:6d}] "
        f"Depth: {telem['depth_m']:.2f} m  "
        f"Water: {telem['water_temp_c']:.2f} C  "
        f"Battery: {telem['battery_v']:.2f} V  "
        f"Inside: {telem['inside_temp_c']:.2f} C  "
        f"Heading: {telem['heading_deg']:.2f} deg  "
        f"Roll: {telem['roll_deg']:.2f} deg  "
        f"Pitch: {telem['pitch_deg']:.2f} deg  "
        f"Leak: {'YES' if telem['leak'] else 'no'}  "
        f"Faults: {faults}  "
        f"Lost (checksum): {reader.checksum_errors}"
    )


def report_state_change(prev, telem):
    """Prints leak / fault changes as soon as they happen."""
    if prev is None:
        return
    if telem["leak"] != prev["leak"]:
        if telem["leak"]:
            print("!!! LEAK SENSOR WET -- STM32 has shut down thrusters and tilt servo !!!")
        else:
            print("*** Leak sensor dry again -- type 'r' + Enter to recover ***")

    was_shutdown = bool(prev["fault_flags"] & FAULT_LEAK)
    is_shutdown = bool(telem["fault_flags"] & FAULT_LEAK)
    if was_shutdown and not is_shutdown:
        print("*** Leak shutdown cleared -- STM32 back to normal operation ***")

    if telem["fault_flags"] != prev["fault_flags"]:
        faults = ", ".join(telem["faults"]) or "none"
        print(f"*** Faults changed: {faults} ***")


def rx_loop():
    last_print = 0.0
    frames = 0
    prev_telem = None

    connected = False                  # True while valid telemetry keeps arriving
    last_rx = time.monotonic()         # time of the last valid telemetry frame
    waiting_since = last_rx            # when we started waiting (for the elapsed-time message)
    last_wait_print = float("-inf")    # so the first waiting message isn't delayed

    while not stop_event.is_set():
        data = ser.read(64)            # returns b"" after the 0.1 s timeout if nothing came in

        # Only VALID frames (START + checksum ok) count as a connection --
        # random noise on an unplugged wire is rejected by the reader.
        new_frames = list(reader.feed(data)) if data else []
        now = time.monotonic()

        if new_frames:
            last_rx = now
            if not connected:
                connected = True
                print(f"*** Connected to STM32 -- telemetry received "
                      f"(waited {now - waiting_since:.1f} s) ***")

            for telem in new_frames:
                frames += 1

                # Leak / fault changes are reported right away, regardless of print rate.
                report_state_change(prev_telem, telem)
                prev_telem = telem

                if now - last_print >= PRINT_PERIOD:
                    print_telemetry(frames, telem)
                    last_print = now

        elif now - last_rx > LINK_TIMEOUT:
            if connected:
                # Was connected, now nothing for LINK_TIMEOUT -> report once.
                connected = False
                prev_telem = None
                waiting_since = last_rx
                last_wait_print = now
                print(f"*** Connection lost -- no telemetry for {LINK_TIMEOUT} s ***")

            if now - last_wait_print >= WAIT_PRINT_PERIOD:
                last_wait_print = now
                print(f"Waiting for connection to STM32... "
                      f"({now - waiting_since:.0f} s)")


def key_loop():
    """Reads simple keyboard commands (letter + Enter)."""
    while not stop_event.is_set():
        try:
            line = input().strip().lower()
        except EOFError:
            return                     # no terminal (e.g. run from a service)
        if line == "r":
            print(">>> Sending recover command")
            recover_request.set()
        elif line == "q":
            stop_event.set()


rx_thread = threading.Thread(target=rx_loop, daemon=True)
key_thread = threading.Thread(target=key_loop, daemon=True)

try:
    rx_thread.start()
    key_thread.start()
    print("Type 'r' + Enter to recover from a leak shutdown, 'q' + Enter to quit.")

    recover_left = 0
    next_t = time.monotonic()
    while not stop_event.is_set():
        if recover_request.is_set():
            recover_request.clear()
            recover_left = RECOVER_FRAMES

        # recover=1 for RECOVER_FRAMES frames, then back to 0, so the next
        # request is another 0 -> 1 edge.
        recover = recover_left > 0
        if recover_left > 0:
            recover_left -= 1

        # One complete frame per write() so it goes out as one burst.
        # NOTE: commands are sent even while waiting -- the STM32 applies them
        # as soon as it boots, so these motor values take effect right away.
        # During a leak shutdown the STM32 ignores motors/tilt and only
        # looks at the recover byte.
        ser.write(build_cmd_frame(motors=TEST_MOTORS,
                                  cam_tilt_deg=TEST_TILT_DEG,
                                  recover=recover))

        # Fixed-period scheduling: doesn't drift the way sleep(0.02) would.
        next_t += CMD_PERIOD
        time.sleep(max(0.0, next_t - time.monotonic()))

except KeyboardInterrupt:
    pass

finally:
    print("\nStopping")
    stop_event.set()
    # Stop frame: thrusters 0 %, camera level. (The STM32 would also stop the
    # thrusters by itself 150 ms after commands stop, but this is immediate.)
    ser.write(build_cmd_frame(motors=[0, 0, 0, 0, 0, 0], cam_tilt_deg=0.0))
    ser.flush()   # make sure the stop frame actually leaves before closing
    rx_thread.join(timeout=0.5)
    ser.close()