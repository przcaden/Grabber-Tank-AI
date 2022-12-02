
#########################################################################
#                                                                       #
#                        Grabber-Tank AI Script                         #
#                              Caden Perez                              #
#                         CSCE-480 Intro to AI                          #
#                                                                       #
#           Main logic function for the robot's AI program.             #
#                                                                       #
#########################################################################


import adeeptlib.move as move
import adeeptlib.RPIservo as RPIservo
import sense
import cv2
import RPi.GPIO as GPIO
import io
import socket
import struct
from time import time
from time import sleep
import threading
import numpy
from picamera import PiCamera
from PIL import Image

# Connect client to PC over local wifi (must be the same network/IPV4)
IPV4 = '172.17.43.0'
port = 5000
client_socket = socket.socket()
client_socket.connect((IPV4, port))
connection = client_socket.makefile('wb')

# Set GPIO input/output modes
move.setup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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
cam.framerate = 32
cam.rotation = 0
cam.hflip = False
cam.vflip = True
cam.resolution = (500, 480)

# Gather data for Haar Cascading image recognition
object_data = cv2.CascadeClassifier('assets/object_cascade.xml')

# Reset arm to neutral position
def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    P_sc.initConfig(1,init_pwm1,1)
    T_sc.initConfig(2,init_pwm2,1)
    H_sc.initConfig(3,init_pwm3,1)
    G_sc.initConfig(4,init_pwm4,1)

# Path-finding class
class Path:
    def __init__(self):
        self.directions = [ [-1,0], [0,1], [1,0], [0,-1] ]
        self.visited = {}
        self.times = []
        self.turns = []

    # Log a new time between turns
    def newTime(self, base_time):
        cur_time = time()
        self.times.append(cur_time - base_time)
        return cur_time

    # Log a new turn
    def newTurn(self, t):
        self.turns.append(t)

    # Detect if an object or wall is in view and decide what action to take
    def wallDetected(detected_objects, img):
        dis = sense.ultra() # distance in cm

        if dis < 3 and dis > 1:
            move.motorStop() # stop moving
            if len(detected_objects) > 0:
                max_size = 0
                for x in detected_objects:
                    if x.size() > max_size:
                        closest_object = x
                x = closest_object.x
                w = closest_object.w

                # Object detected, change direction to approach object
                if x > (img.x/2) + w:
                    return 'redirect_left'
                if x < (img.x/2) - w:
                    return 'redirect_right'

                # If object is centered, opt to grab it
                elif x >= (img.x/2) - w and x <= (img.x/2) + w:
                    return 'grab'

            # Wall detected
            else:
                return 'wall'

        # If no object or wall detected, carry on
        return 'none'


# Receives image data from the established stream with the PC client.
def stream_request(stream):
    # Write length of capture to the stream and flush to ensure it's sent
    connection.write(struct.pack('<L', stream.tell()))
    connection.flush()
    # Rewind stream and receive image
    stream.seek(0)
    # img = stream.read()
    # return img

def grab_sequence(img):
    grab = 0

def drop_sequence():
    arm_lower = 0
    hand_release = 0
    servoPosInit()

####################################### MAIN LOGIC METHOD #######################################

def main_logic():
    # Reset arm to neutral position
    servoPosInit()

    # Start preview and warm up camera for 2 seconds
    cam.start_preview()
    sleep(2)

    # Construct a stream to hold image data
    stream = io.BytesIO()

    # Declare algorithm variables
    path = Path()
    object_in_hand = False
    goal_finish = False
    verify = False
    arrow = 0
    row = 0
    col = 0
    speed_set = 30

    rawCapture = 
    
    for frame in cam.capture_continuous(rawCapture, 'jpeg'):
        print('test')
        img = frame.array
        # img = stream_request(stream)
        cv2.imshow('img', img)
        # num_bytes = len(img)

        # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Send image data to client
        # img_gray = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
        # connection.write(img_gray)

        # Reset the stream for the next capture
        # stream.seek(0)
        # stream.truncate()
    move.motorStop()
    print('finished first loop')

    # Main AI running block
    base_time = time()
    dfs_time = time()
    while not verify:
        # Run continuous stream of video from RPi camera
        for frame in cam.capture_continuous(stream, 'jpeg'):
            # Get image from RPi camera and detect if any objects are in view
            img = stream_request(stream)
            img_data = frame.array
            cv2.imshow(img_data)
            img_gray = cv2.cvtColor(numpy.array(img), cv2.COLOR_BGR2GRAY)
            detected_objects = object_data.detectMultiScale(img_gray, minSize=(20, 20))

            # Highlight any found objects in the image
            for (x,y,w,h) in detected_objects:
                img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

            # Create a status flag depending on distance from detected objects or walls
            status = path.wallDetected(detected_objects, img)

            # Generate string representation for node in path
            key = str(row) + ',' + str(col)

            # Perform decisionmaking based on the status flag if an object has not been picked up
            if not object_in_hand:
                # Check if node has been traversed (DFS) and robot has traveled for 0.1 seconds
                if key not in path.visited and time() > dfs_time + 0.1:
                    path.visited.add(key)
                    
                    if status == 'redirect_left':
                        base_time = path.newTime(base_time)
                        move.move(speed_set, 'no', 'left', 0.25)
                        arrow = (arrow-1) % 4

                    elif status == 'redirect_right':
                        base_time = path.newTime(base_time)
                        move(speed_set, 'no', 'right', 0.25)
                        arrow = (arrow+1) % 4

                    elif status == 'wall':
                        move.motorStop()
                        while status == 'wall':
                            move(speed_set, 'no', 'right', 0.25)
                            arrow = (arrow+1) % 4
                            status = path.wallDetected()

                    elif status == 'grab':
                        move.motorStop()
                        grab_sequence = 0
                        base_time = path.times.pop()

                elif key in path.visited:
                    move.motorStop()
                    move(speed_set, 'no', 'right', 0.25)
                    arrow = (arrow+1) % 4

                # Update tracked direction faced by robot
                curDirection = path.directions[arrow]
                row += curDirection[0]
                col += curDirection[1]

            # Perform decisionmaking for backtracking if the object has been picked up
            else:
                if status == 'wall' or time.time() - base_time <= 0.1:
                    move.motorStop()

                    # If path has been fully backtracked (is empty) and time has elapsed, place the object back down
                    if not path.turns:
                        drop_sequence = 0
                        goal_finish = True

                    # Otherwise, proceed in backtracking path
                    else:
                        turn = path.turns.pop()
                        base_time = path.times.pop()

                        # Reverse direction of the original path, since we are moving backwards
                        if turn == 'L':
                            move(speed_set, 'no', 'right', 0.25)
                        else: move(speed_set, 'no', 'left', 0.25)

                # Continue moving forward otherwise
                else:
                    move(speed_set, 'forward', 'no', 0)

            # Send image data to client
            connection.write(img)

            # Reset the stream for the next capture
            stream.seek(0)
            stream.truncate()

        # Check if camera capture ended prematurely
        if not goal_finish:
            stream = io.BytesIO()
        else: verify = True

    # Write a length of zero to the stream to signal algorithm is done
    connection.write(struct.pack('<L', 0))

    # Code cleanup before closing program
    connection.close()
    client_socket.close()
    move.destroy()


if __name__ == '__main__':
    main_logic()
    print ('Hello juicy Wrld')
