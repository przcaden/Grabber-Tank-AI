
#########################################################################
#                                                                       #
#                        Grabber-Tank AI Script                         #
#                              Caden Perez                              #
#                         CSCE-480 Intro to AI                          #
#                                                                       #
#           Main logic function for the robot's AI program.             #
#                                                                       #
#########################################################################


import rpilib.move as move
import rpilib.RPIservo as RPIservo
import rpilib.ledstrip as ledstrip
import dfs
import sense
import cv2
import RPi.GPIO as GPIO
import io
import socket
import struct
import time
import threading
import Adafruit_PCA9685
from picamera import PiCamera

# Connect client to PC over local wifi (must be the same network/IPV4)
IPV4 = '192.168.69.108'
port = 5000
client_socket = socket.socket()
client_socket.connect((IPV4, port))
connection = client_socket.makefile('wb')

# Set GPIO input/output modes
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

# Gather data for AI training / image comparisons
# img = cv2.imread('object.jpg')
# img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# object_data = cv2.CascadeClassifier('object_data.xml')

# Reset arm to neutral position
def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    P_sc.initConfig(1,init_pwm1,1)
    T_sc.initConfig(2,init_pwm2,1)
    H_sc.initConfig(3,init_pwm3,1)
    G_sc.initConfig(4,init_pwm4,1)

# Receives image data from the established stream to the PC client.
def stream_request(stream):
    # Write length of capture to the stream and flush to ensure it's sent
    connection.write(struct.pack('<L', stream.tell()))
    connection.flush()
    # Rewind stream and receive image
    stream.seek(0)
    img = stream.read()
    print('img type: ' + str(type(img)))
    return img


####################################### MAIN METHOD ####################################### 

def main_logic():
    # Reset arm to neutral position
    servoPosInit()

    # Start preview and warm up camera for 2 seconds
    cam.start_preview()
    time.sleep(2)

    # Construct a stream to hold image data
    stream = io.BytesIO()

    # Declare algorithm variables
    path = dfs.Path()
    led = ledstrip.LED()
    object_in_hand = False
    goal_finish = False
    verify = False
    arrow, row, col = 0
    speed_set = 30
    for foo in cam.capture_continuous(stream, 'jpeg'):
        print('test')
        img = stream_request(stream)
        # Send image data to client
        connection.write(img)

        # Reset the stream for the next capture
        stream.seek(0)
        stream.truncate()
    print('finished first loop')

    # # Main AI running block
    # base_time = time.time()
    # while not verify:
    #     # Run continuous stream of video from RPi camera
    #     for foo in cam.capture_continuous(stream, 'jpeg'):
    #         # Get image from RPi camera and detect if any objects are in view
    #         img = stream_request(stream)
    #         cv2.imshow(img)
    #         img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #         detected_objects = object_data.detectMultiScale(img_gray, minSize=(20, 20))

    #         # Highlight any found objects in the image
    #         for (x,y,w,h) in detected_objects:
    #             img = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

    #         # Create a status flag depending on distance from detected objects or walls
    #         status = path.wallDetected(detected_objects, img)

    #         # Perform decisionmaking based on the status flag if an object has not been picked up
    #         if not object_in_hand:
    #             if len(detected_objects) > 0:
    #                 led.colorWipe(255, 0, 0) # display red LED

    #             # No wall or object detected, continue moving forward
    #             if status == 'none':
    #                 # turn off LED
    #                 row, col = path.dfs(row, col, arrow, detected_objects, img)
                    
    #             elif status == 'redirect_left':
    #                 path.newTime(base_time)
    #                 move.move(speed_set, 'no', 'left', 0.25)
    #                 approaching = True

    #             elif status == 'redirect_right':
    #                 cur_time = time.time()
    #                 base_time = path.newTime(cur_time, base_time)
    #                 move(speed_set, 'no', 'right', 0.25)
    #                 approaching = True

    #             elif status == 'wall':
    #                 # this section will require some sort of DFS or decisionmaking
    #                 move.motorStop()

    #             elif status == 'grab' and not object_in_hand:
    #                 led.colorWipe(0, 0, 255) # display blue LED
    #                 grab_sequence = 0

    #                 base_time = path.times.pop()

    #         # Perform decisionmaking for backtracking if the object has been picked up
    #         else:
    #             led.colorWipe(0, 255, 0) # display green LED
    #             if status == 'wall' or time.time() - base_time <= 0.1:
    #                 move.motorStop()

    #                 # If path has been fully backtracked (is empty) and time has elapsed, place the object back down
    #                 if not path.turns:
    #                     drop_sequence = 0
    #                     goal_finish = True

    #                 # Otherwise, proceed in backtracking path
    #                 else:
    #                     turn = path.turns.pop()
    #                     base_time = path.times.pop()

    #                     # Reverse direction of the original path, since we are moving backwards
    #                     if turn == 'L':
    #                         move(speed_set, 'no', 'right', 0.25)
    #                     else: move(speed_set, 'no', 'left', 0.25)

    #             # Continue moving forward otherwise
    #             else:
    #                 move(speed_set, 'forward', 'no', 0)

    #         # Send image data to client
    #         connection.write(img)

    #         # Reset the stream for the next capture
    #         stream.seek(0)
    #         stream.truncate()

    #     # Check if camera capture ended prematurely
    #     if not goal_finish:
    #         stream = io.BytesIO()
    #     else: verify = True

    # # Display party lights
    # time.sleep(2)
    # base_time = time.time()
    # while time.time() < base_time+10:
    #     led.colorWipe(255, 0, 0) # display red LED
    #     time.sleep(0.5)
    #     led.colorWipe(0, 0, 255) # display blue LED
    #     time.sleep(0.5)
    #     led.colorWipe(0, 255, 0) # display green LED
    #     time.sleep(0.5)

    # # Write a length of zero to the stream to signal we're done
    # connection.write(struct.pack('<L', 0))

    # # Code cleanup before closing program
    # connection.close()
    # client_socket.close()
    # move.destroy()


if __name__ == '__main__':
    main_logic()