
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
# Following block is the boot sequence. Pre-generated data is used to
# compare with image values that will be received from the RPi camera.
# Motors and sensors will be initialized and calibrated.

# Define GPIO pins
gear_left_pin = 1
gear_right_pin = 2
servo1_pin = 1
servo2_pin = 2
servo3_pin = 3
servo4_pin = 4
servo5_pin = 5
trigger = 6
echo = 7

# Set GPIO input/output modes
GPIO.setmode(GPIO.BCM)
GPIO.setup(gear_left_pin, GPIO.OUT)
GPIO.setup(gear_right_pin, GPIO.OUT)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)
GPIO.setup(servo3_pin, GPIO.OUT)
GPIO.setup(servo4_pin, GPIO.OUT)
GPIO.setup(servo5_pin, GPIO.OUT)
GPIO.setup(trigger, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

# Define motor variables
servo1 = GPIO.PWM(servo1_pin, 50) # 50 Hz PWM signal
servo2 = GPIO.PWM(servo2_pin, 50)
servo3 = GPIO.PWM(servo3_pin, 50)
servo4 = GPIO.PWM(servo4_pin, 50)
servo5 = GPIO.PWM(servo5_pin, 50)
servo1.start(0)
servo2.start(0)
servo3.start(0)
servo4.start(0)
servo5.start(0)

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


# Set the angle of a given servo motor using its duty cycle
def setAngle(motor_pin, motor, angle):
    duty = angle / 18 + 2
    GPIO.output(motor_pin, True)
    motor.ChangeDutyCycle(duty)
    utime.sleep(1)
    GPIO.output(motor_pin, False)
    motor.ChangeDutyCycle(duty)


# Reset arm to neutral position
def resetArm():
    setAngle(servo1_pin, servo1, 0)
    setAngle(servo2_pin, servo2, 0)
    setAngle(servo3_pin, servo3, 0)
    setAngle(servo4_pin, servo4, 0)
    setAngle(servo5_pin, servo5, 0)

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

# Clean up GPIO code for when the robot terminates the program
def cleanup():
    servo1.stop()
    servo2.stop()
    servo3.stop()
    servo4.stop()
    servo5.stop()
    GPIO.cleanup()