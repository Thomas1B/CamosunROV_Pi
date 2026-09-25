#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3
"""
Test script for UART_DMA: sends command frames at 50 Hz and prints
telemetry coming back from the STM32.

While no valid telemetry is arriving (STM32 off, booting, or unplugged), it
prints a "waiting for connection" message every WAIT_PRINT_PERIOD seconds.
It reports once when the connection comes up, and once if it is lost.

Run it directly (uses the venv via the shebang; needs chmod +x once):
    ./UART_DMA_TEST.py
or with the venv's interpreter explicitly:
    ~/CamosunROV_Pi/ROV_venv/bin/python3 UART_DMA_TEST.py
or activate the venv first and run:
    source ~/CamosunROV_Pi/ROV_venv/bin/activate
    python3 UART_DMA_TEST.py
"""

import serial
import time
import threading

from UART_DMA import build_cmd_frame, TelemetryReader

CMD_PERIOD = 1/50        # s -> 50 Hz command rate
PRINT_PERIOD = 1/2       # s -> print telemetry at 2 Hz so the terminal stays readable

LINK_TIMEOUT = 0.5       # s without a valid telemetry frame -> treated as not connected
                         # (telemetry normally arrives every 0.02 s, so 0.5 s = 25 missed frames)
WAIT_PRINT_PERIOD = 2.0  # s between "waiting for connection" messages

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


def print_telemetry(frames, telem):
    faults = ", ".join(telem["faults"]) or "none"
    print(
        f"[{frames:6d}] "
        f"Depth: {telem['depth_m']:.1f} m  "
        f"Water: {telem['water_temp_c']:.1f} C  "
        f"Battery: {telem['battery_v']:.1f} V  "
        f"Inside: {telem['inside_temp_c']:.1f} C  "
        f"Heading: {telem['heading_deg']:.1f} deg  "
        f"Roll: {telem['roll_deg']:.1f} deg  "
        f"Pitch: {telem['pitch_deg']:.1f} deg  "
        f"Leak: {'YES' if telem['leak'] else 'no'}  "
        f"Faults: {faults}"
    )


def rx_loop():
    last_print = 0.0
    frames = 0

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

                # Always report a live leak immediately, regardless of print rate.
                if now - last_print >= PRINT_PERIOD:
                    if telem["leak"]:
                        print("!!! LEAK DETECTED !!!")
                    else:
                        print_telemetry(frames, telem)
                    last_print = now

        elif now - last_rx > LINK_TIMEOUT:
            if connected:
                # Was connected, now nothing for LINK_TIMEOUT -> report once.
                connected = False
                waiting_since = last_rx
                last_wait_print = now
                print(f"*** Connection lost -- no telemetry for {LINK_TIMEOUT} s ***")

            if now - last_wait_print >= WAIT_PRINT_PERIOD:
                last_wait_print = now
                print(f"Waiting for connection to STM32... "
                      f"({now - waiting_since:.0f} s)")


rx_thread = threading.Thread(target=rx_loop, daemon=True)

try:
    rx_thread.start()

    next_t = time.monotonic()
    while True:
        # One complete frame per write() so it goes out as one burst.
        # NOTE: commands are sent even while waiting -- the STM32 applies them
        # as soon as it boots, so these motor values take effect right away.
        ser.write(build_cmd_frame(motors=[50, 0, -100, 30, 0, 0],
                                  cam_tilt_deg=-15.0))

        # Fixed-period scheduling: doesn't drift the way sleep(0.02) would.
        next_t += CMD_PERIOD
        time.sleep(max(0.0, next_t - time.monotonic()))

except KeyboardInterrupt:
    print("\nStopping")
    ser.write(build_cmd_frame(motors=[0, 0, 0, 0, 0, 0], cam_tilt_deg=0.0))
    ser.flush()   # make sure the stop frame actually leaves before closing

finally:
    stop_event.set()
    rx_thread.join(timeout=0.5)
    ser.close()