
import cv2
import os
import RPi.GPIO as GPIO
import picamera
import time

# May change later to fit os res
CAM_WIDTH = 640
CAM_HEIGHT = 480

# Initialize Raspberry Pi camera
cam = PiCamera()
cam.resolution = (CAM_WIDTH, CAM_HEIGHT)
cam.framerate = 32
rawCapture = PiGRBArray(cam, size=(CAM_WIDTH, CAM_HEIGHT))

# Allow camera calibration
time.sleep(0.1)

# Display the video image
for frame in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    result, img = cam.read()
    if result:
        cv2.imshow('Vid Image', img)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    if key == ord("q"):
        break