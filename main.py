#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3
"""
CamosunROV -- Raspberry Pi main program. This is the one program to run.

Starts the two parts of the ROV software, each in its own process:
    bridge  (rov/bridge.py)  command centre <-> STM32: thrusters, tilt, telemetry, failsafe
    camera  (rov/camera.py)  Pi camera -> H.264 video stream to the command centre

Separate processes so a camera crash or hang can never stop thruster control,
and video work can't disturb the 50 Hz STM32 timing. They share one flag,
video_enabled: the bridge clears it on a leak and the camera stops streaming
until a recover command clears the leak shutdown. If either part stops,
main.py restarts it (after 2 s, backing off to 10 s if it keeps failing);
the other part keeps running.

Console output: one stream, every line tagged [MAIN], [BRIDGE] or [CAMERA].
The bridge prints like Test_UART_DMA.py: waiting / connected / lost messages,
telemetry and command lines at 2 Hz, leak and fault changes straight away.

Settings (IPs, ports, rates, camera) are all in rov/settings.py.

Run it directly (uses the venv via the shebang; needs chmod +x once):
    ./main.py
Stop: Ctrl+C (sends the STM32 a stop frame on the way out).

At boot it runs as the 'rov' systemd service (systemd/rov.service); then
    sudo systemctl stop rov        before running ./main.py or Test_UART_DMA.py by hand
    journalctl -u rov -f           to watch the same console output live

Only one program can use /dev/serial0: don't run Test_UART_DMA.py while
main.py (or the rov service) is running.
"""

import multiprocessing as mp
import signal
import sys
import time

from rov import bridge, camera
from rov import settings as cfg

PARTS = {
    "bridge": bridge.run,
    "camera": camera.run,
}
STABLE_S = 30.0          # a part that ran this long before stopping gets the short restart delay again
STOP_TIMEOUT_S = 3.0     # wait this long for each part to shut down cleanly, then kill it


def say(msg):
    # One write per line, so lines from the bridge, camera and main processes
    # never get mixed together on the shared console.
    sys.stdout.write(f"[MAIN]   {msg}\n")
    sys.stdout.flush()


class Part:
    def __init__(self, ctx, name, target, args=()):
        self.ctx, self.name, self.target, self.args = ctx, name, target, args
        self.proc = None
        self.started = 0.0
        self.delay = cfg.RESTART_DELAY_MIN
        self.restart_at = None

    def start(self):
        self.proc = self.ctx.Process(target=self.target, args=self.args,
                                     name=self.name, daemon=False)
        self.proc.start()
        self.started = time.monotonic()
        self.restart_at = None

    def check(self, now):
        """Restart the part if it has stopped."""
        if self.restart_at is not None:
            if now >= self.restart_at:
                say(f"Restarting {self.name}")
                self.start()
            return
        if self.proc.is_alive():
            return
        code = self.proc.exitcode
        if now - self.started >= STABLE_S:
            self.delay = cfg.RESTART_DELAY_MIN
        say(f"!!! {self.name} stopped (exit code {code}) -- restarting in {self.delay:.0f} s")
        self.restart_at = now + self.delay
        self.delay = min(self.delay * 2, cfg.RESTART_DELAY_MAX)

    def stop(self):
        p = self.proc
        if p is None or not p.is_alive():
            return
        p.terminate()                      # SIGTERM -> the part shuts down cleanly
        p.join(STOP_TIMEOUT_S)
        if p.is_alive():
            say(f"{self.name} did not stop in {STOP_TIMEOUT_S:.0f} s -- killing it")
            p.kill()
            p.join(1.0)


def main():
    # systemctl stop sends SIGTERM: turn it into the same clean exit as Ctrl+C.
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    say("CamosunROV Pi starting -- Ctrl+C to stop")
    # "spawn": each part starts as a fresh Python process (nothing inherited from this one).
    ctx = mp.get_context("spawn")
    # Leak video cut-off, shared by both parts. Set = camera may stream.
    # The bridge clears it while the STM32 is in leak shutdown and sets it again
    # once a recover command has cleared the shutdown; the camera follows it.
    # It lives here so it survives either part being restarted.
    video_enabled = ctx.Event()
    video_enabled.set()
    parts = [Part(ctx, name, target, args=(video_enabled,)) for name, target in PARTS.items()]
    try:
        for part in parts:
            part.start()
        while True:
            time.sleep(0.5)
            now = time.monotonic()
            for part in parts:
                part.check(now)
    except KeyboardInterrupt:
        pass
    finally:
        # Ignore further Ctrl+C / SIGTERM while shutting down, so the bridge
        # always gets to send its stop frame.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        say("Stopping...")
        for part in parts:                 # bridge first: thrusters stopped before anything else
            part.stop()
        say("Stopped")


if __name__ == "__main__":
    sys.exit(main())