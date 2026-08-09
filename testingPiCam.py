#!/home/pirov/CamosunROV_Pi/ROV_venv/bin/python3

'''
Testing pi camera


in the venv, run "pip install -r requiredPackages.txt"

libcamera must be installed to system use:
sudo apt update
sudo apt install -y python3-libcamera python3-picamera2 --no-install-recommends
'''

from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)

picam2.start_preview(Preview.QTGL)  # opens a live preview window
picam2.start()

time.sleep(30)  # keep preview open for 30s

picam2.stop()
picam2.stop_preview()

