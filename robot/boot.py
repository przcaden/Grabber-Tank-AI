
import cv2
import os
import RPi.GPIO as GPIO
import picamera
import time
import numpy as np

# Define pins and sensors
cam = PiCamera()
cam.framerate = 32

# ---------------------------- CALIBRATION ---------------------------- #
# Following block is the calibration sequence. Pre-generated data is used
# to compare with image values that will be received from the RPi camera.
# Motors and sensors will be initialized and calibrated.

# Gather data for AI training / image comparisons
img = cv2.imread('object.jpg')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
object_data = cv2.CascadeClassifier('object_data.xml')
found = object_data.detectMultiScale(img_gray, minSize =(20, 20))

# Calibrate motors

# Calibrate ultrasonic sensor