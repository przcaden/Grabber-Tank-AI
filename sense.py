
#########################################################################
#                                                                       #
#                       Grabber-Tank Sensor Script                      #
#                              Caden Perez                              #
#                         CSCE-480 Intro to AI                          #
#                                                                       #
#               Defines usage functions for the ultrasonic              #
#                       and path-tracking sensors.                      #
#                                                                       #
#########################################################################

import RPi.GPIO as GPIO
import time

Tr = 11
Ec = 8
line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20

# Initialize sensors' GPIO pins
def sensor_setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)

# Get statuses of the tracking modules
def tracking():
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    return (status_right, status_middle, status_left)

# Fetch ultrasonic sensor reading, prints distance in cm
def ultra():
    for i in range(5):  # remove invalid test results
        GPIO.output(Tr, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Tr, GPIO.LOW)
        
        while not GPIO.input(Ec):
            pass
        t1 = time.time()
        while GPIO.input(Ec):
            pass
        t2 = time.time()
        dist = (t2-t1)*340/2

        # Check if past 5 iterations have valid data
        if dist > 9 and i < 4:
            continue
        else:
            return (t2-t1)*340/2


