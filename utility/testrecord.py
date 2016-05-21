import picamera
from time import sleep
camera=picamera.PiCamera()
camera.start_recording('testvideo_longer.h264')
sleep(60)
camera.stop_recording()
