
#########################################################################
#                                                                       #
#                       Grabber-Tank Init Script                        #
#                              Caden Perez                              #
#                         CSCE-480 Intro to AI                          #
#                                                                       #
#    Boot sequence: Pre-generated data is used to compare with image    #
#    values that will be received from the RPi camera. Motors and       #
#               sensors will be initialized and calibrated.             #
#                                                                       #
#########################################################################


import cv2
import rpilib.RPIservo as RPIservo
import RPi.GPIO as GPIO
from picamera import PiCamera
import PiRGBArray
import utime

# Define GPIO pins
gear_left_pin = 1
gear_right_pin = 2
servo1_pin = 1
servo2_pin = 2
servo3_pin = 3
servo4_pin = 4
servo5_pin = 5

# Set GPIO input/output modes
GPIO.setmode(GPIO.BCM)

# Define servo variables
scGear = RPIservo.ServoCtrl()
scGear.moveInit()
P_sc = RPIservo.ServoCtrl()
P_sc.start()
T_sc = RPIservo.ServoCtrl()
T_sc.start()
H_sc = RPIservo.ServoCtrl()
H_sc.start()
G_sc = RPIservo.ServoCtrl()
G_sc.start()

init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]
init_pwm4 = scGear.initPos[4]

# Initialize camera
cam = PiCamera()
rawCapture = PiRGBArray(cam, size=(640, 480))
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

# Reset arm to neutral position
def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    P_sc.initConfig(1,init_pwm1,1)
    T_sc.initConfig(2,init_pwm2,1)
    H_sc.initConfig(3,init_pwm3,1)
    G_sc.initConfig(4,init_pwm4,1)
