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

Leak cut-off: while the shared video_enabled flag (main.py) is clear, which the
bridge does during a leak shutdown, the stream is stopped and the camera
closed. It reopens when a recover command clears the shutdown.
"""

import os
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


LEAK_POLL_S = 0.2        # s   how often the stream loop checks the leak video flag


def video_allowed(video_enabled):
    return video_enabled is None or video_enabled.is_set()


def stream_once(video_enabled=None):
    """Streams until an error (raises) or a leak cut-off (returns normally)."""
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

        # Frame watchdog. The camera can "start" without ever delivering a frame:
        # e.g. the HQ camera at boot, when unicam logs "Failed to start media
        # pipeline: -22" in dmesg. picamera2 raises no error then, so we count
        # frames ourselves and treat "no frames for a while" as a failure.
        last_frame = [time.monotonic()]
        frames = [0]

        def on_frame(request):
            last_frame[0] = time.monotonic()
            frames[0] += 1
        picam2.post_callback = on_frame

        w, h = cfg.VIDEO_SIZE
        say(f"*** Streaming: {picam2.camera_properties.get('Model', '?')}  {w}x{h} @ {cfg.FPS} fps, "
            f"{cfg.BITRATE / 1e6:.1f} Mbit/s -> {cfg.LAPTOP_IP}:{cfg.VIDEO_PORT} ***")
        picam2.start_recording(encoder, output)
        recording = True
        last_frame[0] = time.monotonic()
        while not failed.wait(LEAK_POLL_S):
            if not video_allowed(video_enabled):
                say("!!! LEAK -- stopping video stream (camera closed until a recover clears it) !!!")
                return                        # finally: stop_recording() + close()
            waited = time.monotonic() - last_frame[0]
            limit = cfg.CAMERA_STALL_TIMEOUT if frames[0] else cfg.CAMERA_START_TIMEOUT
            if waited > limit:
                what = "stopped sending frames" if frames[0] else "never sent a frame"
                say(f"!!! Camera {what} ({waited:.0f} s) -- restarting the camera process")
                # A stalled camera can also hang stop_recording()/close(), so skip
                # the clean-up and exit: main.py starts a fresh camera process in
                # 2 s, and the kernel releases the camera when this one exits.
                os._exit(3)
        raise RuntimeError("stream output failed")
    finally:
        try:
            if recording:
                picam2.stop_recording()
        finally:
            picam2.close()


def wait_for_video_allowed(video_enabled):
    """Blocks while the leak cut-off is active. Short waits so Ctrl+C / SIGTERM still work."""
    if video_allowed(video_enabled):
        return
    say("Video off (leak) -- waiting for a recover command to clear the leak shutdown")
    while not video_enabled.wait(0.5):
        pass
    say("*** Leak shutdown cleared -- restarting video stream ***")


def run(video_enabled=None):
    """Process entry point used by main.py. Keeps retrying if the camera or network hiccups.

    video_enabled: shared flag from main.py, cleared by the bridge during a leak
    shutdown. While it is clear the camera is closed and nothing is streamed.
    """
    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)

    say("Started")
    last_error = None
    while True:
        try:
            wait_for_video_allowed(video_enabled)
            stream_once(video_enabled)
            last_error = None                 # stopped for a leak, not an error
            continue
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