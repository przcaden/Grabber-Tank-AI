
#########################################################################
#                                                                       #
#                         Grabber-Tank Boot Script                      #
#                              Caden Perez                              #
#                                                                       #
#                          CSCE-480 Intro to AI                         #
#                              Final Project                            #
#                                                                       #
#########################################################################


import cv2
import RPi.GPIO as GPIO
from picamera import PiCamera
import utime

# ---------------------------- CALIBRATION ---------------------------- #
# Following block is the booty sequence. Pre-generated data is used to
# compare with image values that will be received from the RPi camera.
# Motors and sensors will be initialized and calibrated.

# Define GPIO pins
gear_left = 1
gear_right = 2
servo1 = 1
servo2 = 2
servo3 = 3
servo4 = 4
servo5 = 5
trigger = 6
echo = 7

# Set GPIO input/output modes
GPIO.setmode(GPIO.BCM)
GPIO.setup(gear_left, GPIO.OUT)
GPIO.setup(gear_right, GPIO.OUT)
GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)
GPIO.setup(servo3, GPIO.OUT)
GPIO.setup(servo4, GPIO.OUT)
GPIO.setup(servo5, GPIO.OUT)
GPIO.setup(trigger, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

# Initialize camera
cam = PiCamera()
cam.framerate = 32
cam.rotation = 0
cam.hflip = False
cam.vflip = True
cam.resolution = (500, 480)

# Gather data for AI training / image comparisons
img = cv2.imread('object.jpg')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
object_data = cv2.CascadeClassifier('object_data.xml')

# Calibrate arm
def resetMotors():
    GPIO.output(servo1, True)

# Define ultrasonic sensor reading, prints distance in cm
def ultra():
    GPIO.output(trigger, False)
    utime.sleep_us(5)
    GPIO.output(trigger, True)
    utime.sleep_us(5)
    GPIO.output(trigger, False)
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    while echo.value() == 1:
        signalon = utime.ticks_us()
    time = signalon - signaloff
    return (time * 0.0343) / 2
