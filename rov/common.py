"""Small helpers shared by main.py, rov/bridge.py and rov/camera.py."""

import signal
import sys


def make_say(tag):
    """Returns a console print function that tags every line,
    e.g. make_say("BRIDGE")("hi") prints "[BRIDGE] hi".

    One write per line, so lines from the bridge, camera and main processes
    never get mixed together on the shared console."""
    prefix = f"[{tag}]".ljust(9)          # "[MAIN]   ", "[BRIDGE] ", "[CAMERA] " line up

    def say(msg):
        sys.stdout.write(f"{prefix}{msg}\n")
        sys.stdout.flush()
    return say


def sigterm_as_ctrl_c():
    """systemctl stop (and main.py stopping a part) sends SIGTERM: turn it into
    the same clean exit as Ctrl+C (KeyboardInterrupt)."""
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)


def ignore_stop_signals():
    """Ignore further Ctrl+C / SIGTERM, so a shutdown already in progress
    (e.g. the bridge's stop frames) can't be interrupted."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
