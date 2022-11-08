
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
import numpy as np
import io
import socket
import struct
import utime

IPV4 = socket.gethostbyname()

# Connect client to PC over local wifi
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

def grab_object(img):
    

def main():
    # Reset arm to neutral position
    bs.resetMotors()

    # Start preview and warm up camera for 2 seconds
    bs.cam.start_preview()
    utime.sleep(2)

    # Construct a stream to hold image data
    start = utime.time()
    stream = io.BytesIO()

    # Run continuous stream of video from RPi camera
    object_in_hand = False
    for foo in bs.cam.capture_continuous(stream, 'jpeg'):

        # Detect if the object is found within the current image
        img = stream_request(stream)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detected_objects = bs.object_data.detectMultiScale(img_gray, minSize=(20, 20))

        # Highlight any found objects in the image
        for (x,y,w,h) in detected_objects:
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

        # Perform decisionmaking for detected objects
        if len(detected_objects) > 0:

            # Run if object has not been picked up
            if not object_in_hand:
                # Calculate closest object (max size)
                max_size = 0
                for x in detected_objects:
                    if x.size() > max_size:
                        closest_object = x

                x = closest_object.x
                w = closest_object.w

                # Attempt to center object within camera view
                if x < (img.x/2) + w:
                    do_something = 0
                if x > (img.x/2) - w:
                    do_something = 0

                # Change movement speed based on ultrasonic distance from object
                if x < img.x/2 < x+w:
                    if bs.ultra() > 5:
                        maintain_constant_speed = 0
                    elif bs.ultra() < 1:
                        reverse_gear_voltage = 0
                    elif bs.ultra > 1 and bs.ultra < 3:
                        stop_dc_gear = 0
                        grab_object(img, closest_object)
                

        # Send image data to client
        connection.write(img)

        # Reset the stream for the next capture
        stream.seek(0)
        stream.truncate()

    # Write a length of zero to the stream to signal we're done
    connection.write(struct.pack('<L', 0))

    # Close connection
    connection.close()
    client_socket.close()