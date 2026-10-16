"""
Live camera stream: Pi camera -> H.264 (Pi 4 hardware encoder) -> MPEG-TS over UDP
to the command centre laptop, which shows it in the console's camera panel.

Started by main.py in its own process, so a camera crash or hang can never
stop thruster control. Every console line is tagged [CAMERA].

Works with the Camera Module v2 (IMX219) now and the HQ Camera (IMX477) later --
picamera2 picks a sensor mode that fits VIDEO_SIZE, so nothing here has to change
when the camera is swapped. Check which camera the Pi sees with:
    rpicam-hello --list-cameras

UDP is "fire and forget": the Pi keeps sending whether or not anyone is
watching, and the laptop can join at any time (SPS/PPS are repeated every
I-frame so the decoder can start mid-stream).
"""

import signal
import sys
import threading
import time

from rov import settings as cfg


def say(msg):
    # One write per line, so lines from the bridge, camera and main processes
    # never get mixed together on the shared console.
    sys.stdout.write(f"[CAMERA] {msg}\n")
    sys.stdout.flush()


def stream_once():
    # Imported here so a missing camera library shows up as a clear [CAMERA]
    # error (and a retry) instead of a crash before anything prints.
    from libcamera import Transform
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import PyavOutput

    picam2 = Picamera2()
    recording = False
    try:
        config = picam2.create_video_configuration(
            main={"size": cfg.VIDEO_SIZE, "format": "YUV420"},
            controls={"FrameRate": cfg.FPS},
            transform=Transform(hflip=cfg.HFLIP, vflip=cfg.VFLIP),
            buffer_count=6,
        )
        picam2.configure(config)

        encoder = H264Encoder(bitrate=cfg.BITRATE, repeat=True, iperiod=cfg.IPERIOD)
        url = f"udp://{cfg.LAPTOP_IP}:{cfg.VIDEO_PORT}?pkt_size=1316"   # 7 x 188-byte TS packets
        output = PyavOutput(url, format="mpegts")
        # PyavOutput stops sending silently if a write fails (it closes its
        # container); this callback lets the loop below notice and restart.
        failed = threading.Event()

        def on_error(exc):
            say(f"Stream write failed: {exc!r}")
            failed.set()
        output.error_callback = on_error

        w, h = cfg.VIDEO_SIZE
        say(f"*** Streaming: {picam2.camera_properties.get('Model', '?')}  {w}x{h} @ {cfg.FPS} fps, "
            f"{cfg.BITRATE / 1e6:.1f} Mbit/s -> {cfg.LAPTOP_IP}:{cfg.VIDEO_PORT} ***")
        picam2.start_recording(encoder, output)
        recording = True
        while not failed.wait(1.0):
            pass
        raise RuntimeError("stream output failed")
    finally:
        try:
            if recording:
                picam2.stop_recording()
        finally:
            picam2.close()


def run():
    """Process entry point used by main.py. Keeps retrying if the camera or network hiccups."""
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    say("Started")
    last_error = None
    while True:
        try:
            stream_once()
        except KeyboardInterrupt:
            say("Stopped")
            return
        except Exception as exc:
            text = repr(exc)
            if text != last_error:            # don't repeat the same error every 2 s
                say(f"Camera error: {text} -- retrying every {cfg.CAMERA_RETRY_DELAY:.0f} s")
                last_error = text
            try:
                time.sleep(cfg.CAMERA_RETRY_DELAY)
            except KeyboardInterrupt:
                say("Stopped")
                return
