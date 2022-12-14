
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

# Initialize sensors' GPIO pins
def sensor_setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)

# Fetch ultrasonic sensor reading, prints distance in cm
def ultra():
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(Tr, GPIO.LOW)
    
    # Save start/stop times of pulses
    while GPIO.input(Ec) == 0:
        StartTime = time.time()
    while GPIO.input(Ec) == 1:
        StopTime = time.time()
        
    # Find duration of pulse and calculate distance
    dur = StopTime - StartTime
    dis = (dur * 34300) / 2
    return dis

# Loop for testing purposes
if __name__ == "__main__":
    sensor_setup()
    while 1:
        print('distance in cm: ' + str(ultra()))
        time.sleep(1)