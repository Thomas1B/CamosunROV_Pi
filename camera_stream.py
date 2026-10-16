#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3
"""
Live camera stream: Pi camera -> H.264 (Pi 4 hardware encoder) -> MPEG-TS over UDP
to the command centre laptop.

Works with the Camera Module v2 (IMX219) now and the HQ Camera (IMX477) later --
picamera2 picks a sensor mode that fits VIDEO_SIZE, so nothing here has to change
when the camera is swapped. Check which camera the Pi sees with:
    rpicam-hello --list-cameras

Watch on the laptop (Windows), lowest latency first:
    ffplay -fflags nobuffer -flags low_delay -framedrop udp://@:5602
    VLC: Media > Open Network Stream > udp://@:5602   (set "Network caching" to ~100 ms)
Allow UDP 5602 inbound in the Windows firewall.

UDP is "fire and forget": the Pi keeps sending whether or not anyone is
watching, and a viewer can join at any time (SPS/PPS are repeated every
I-frame so the decoder can start mid-stream).

Run (needs chmod +x once):
    ./camera_stream.py                     # streams to 192.168.2.1:5602
    ./camera_stream.py --host 192.168.2.1 --port 5602 --size 1280x720 --fps 30
"""

import argparse
import threading
import time

from libcamera import Transform
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput

LAPTOP_IP = "192.168.2.1"      # command centre's static IP on the tether
VIDEO_PORT = 5602              # next to CMD_PORT 5600 / TELEM_PORT 5601
VIDEO_SIZE = (1280, 720)       # px
FPS = 30                       # frames/s
BITRATE = 4_000_000            # bit/s -- ~4 % of the 100 Mbit/s tether link
IPERIOD = 15                   # frames between I-frames (0.5 s at 30 fps): faster recovery from loss
HFLIP = False                  # set True if the image is mirrored once mounted
VFLIP = False                  # set True (with HFLIP) if the camera is mounted upside down
RETRY_DELAY = 2.0              # s before restarting after an error


def parse_size(text):
    w, h = text.lower().split("x")
    return int(w), int(h)


def stream_once(args):
    picam2 = Picamera2()
    recording = False
    try:
        config = picam2.create_video_configuration(
            main={"size": args.size, "format": "YUV420"},
            controls={"FrameRate": args.fps},
            transform=Transform(hflip=HFLIP, vflip=VFLIP),
            buffer_count=6,
        )
        picam2.configure(config)

        encoder = H264Encoder(bitrate=args.bitrate, repeat=True, iperiod=IPERIOD)
        url = f"udp://{args.host}:{args.port}?pkt_size=1316"   # 7 x 188-byte TS packets per datagram
        output = PyavOutput(url, format="mpegts")
        # PyavOutput stops sending silently if a write fails (it closes its
        # container); this callback lets the loop below notice and restart.
        failed = threading.Event()

        def on_error(exc):
            print(f"Stream write failed: {exc!r}", flush=True)
            failed.set()
        output.error_callback = on_error

        print(f"Camera: {picam2.camera_properties.get('Model', '?')}  "
              f"{args.size[0]}x{args.size[1]} @ {args.fps} fps, {args.bitrate / 1e6:.1f} Mbit/s "
              f"-> {url}", flush=True)
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


def main():
    ap = argparse.ArgumentParser(description="Pi camera -> H.264 / MPEG-TS / UDP")
    ap.add_argument("--host", default=LAPTOP_IP)
    ap.add_argument("--port", type=int, default=VIDEO_PORT)
    ap.add_argument("--size", type=parse_size, default=VIDEO_SIZE, help="WxH, e.g. 1280x720")
    ap.add_argument("--fps", type=int, default=FPS)
    ap.add_argument("--bitrate", type=int, default=BITRATE, help="bit/s")
    args = ap.parse_args()

    # Keep going if the camera or network hiccups (e.g. laptop unplugged).
    while True:
        try:
            stream_once(args)
        except KeyboardInterrupt:
            print("\nCamera stream stopped", flush=True)
            return
        except Exception as exc:
            print(f"Camera stream error: {exc!r} -- retrying in {RETRY_DELAY} s", flush=True)
            time.sleep(RETRY_DELAY)


if __name__ == "__main__":
    import signal

    def on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    main()
