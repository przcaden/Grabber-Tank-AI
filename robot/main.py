
#########################################################################
#                                                                       #
#                         Grabber-Tank AI Script                        #
#                              Caden Perez                              #
#                                                                       #
#                          CSCE-480 Intro to AI                         #
#                              Final Project                            #
#                                                                       #
#########################################################################


import boot as bs
import cv2
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import io
import socket
import struct
import utime

IPV4 = socket.gethostbyname()

GPIO.output(bs.servo1, True)
dis = bs.ultra()

# Connect client to PC over local wifi (must be the same network/IPV4)
client_socket = socket.socket()
client_socket.connect((IPV4, 8000))
connection = client_socket.makefile('wb')

# Receives image data from the established stream to the PC client.
def stream_request(stream):
    # Write length of capture to the stream and flush to ensure it's sent
    connection.write(struct.pack('<L', stream.tell()))
    connection.flush()
    # Rewind stream and receive image
    stream.seek(0)
    img = stream.read()
    return img

def main():
    # Reset arm to neutral position
    bs.resetMotors()

    # Start preview and warm up camera for 2 seconds
    bs.cam.start_preview()
    utime.sleep(2)

    # Construct a stream to hold image data
    start = utime.time()
    stream = io.BytesIO()

    goal_finish = False
    while not goal_finish:
        # Run continuous stream of video from RPi camera
        for foo in bs.cam.capture_continuous(stream, 'jpeg'):

            # Get image from RPi camera and detect if any objects are in view
            img = stream_request(stream)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            detected_objects = bs.object_data.detectMultiScale(img_gray, minSize=(20, 20))

            # Highlight any found objects in the image
            for (x,y,w,h) in detected_objects:
                img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

            # Perform AI decisionmaking if an object is detected
            if len(detected_objects) > 0:
                something = 0

            # Send image data to client
            connection.write(img)

            # Reset the stream for the next capture
            stream.seek(0)
            stream.truncate()

        # Check if camera capture ended prematurely
        if not goal_finish:
            bs.resetMotors()
            stream = io.BytesIO()

    # Write a length of zero to the stream to signal we're done
    connection.write(struct.pack('<L', 0))

    # Code cleanup before closing program
    connection.close()
    client_socket.close()
    bs.cleanup()