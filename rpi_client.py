
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
from time import time
from time import sleep
import numpy
from picamera import PiCamera
from picamera.array import PiRGBArray

# Set GPIO input/output modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define servo variables
scGear = RPIservo.ServoCtrl()
scGear.moveInit()

# Declare various servo PWM settings
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
cam.framerate = 30
cam.rotation = 0
cam.hflip = False
cam.vflip = True
CAM_RES = (640,480)
cam.resolution = CAM_RES
rawCapture = PiRGBArray(cam, size=CAM_RES)

# Define bounds and filters for object detection
red_lower = (155,155,100)
red_upper = (179,255,255)
kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))

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
    def wallDetected(self, img, closest_object):
        img_y, img_x = img.shape[:2] # get image dimensions
        dis = sense.ultra() # calculate distance in cm to nearest object
        print(str(dis) + ' cm')
        if closest_object is not None:
            x, y, w, h = cv2.boundingRect(closest_object)
            
            # Obstruction in path, detect if an object is in view
            if 6 < dis < 9:
                # If object is centered, opt to grab it
                if x >= (img_x/2) - w and x <= (img_x/2) + w:
                    return 'grab'
                # Wall detected
                else:
                    return 'wall'
                
            # Object detected, change direction to approach object
            print('img width: ' + str(img_x))
            print('object width: ' + str(w))
            if x > ((img_x/2) - w) * 0.9:
                return 'redirect_left'
            if x < ((img_x/2) + w) * 1.1:
                return 'redirect_right'

        # If no object or wall detected, carry on
        return 'none'
    

# Grab the detected object in view
def grab_sequence(img):
    # distance of object is approx 7 cm
    grab = 0

# Drop the object held by the arm
def drop_sequence():
    arm_lower = 0
    hand_release = 0
    servoPosInit()
    
# Computer vision function: Given a snapshot, detect any red cube-like objects
def findObjects(img):
    # Get image's HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Gets colors in range of red in image
    thresh = cv2.inRange(hsv, red_lower, red_upper)
    
    # Generate morphology filters
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel1)
    clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel2)

    # Get any external contours containg the color red
    contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Detect if detected red object is a cube shape, and grab only the closest object
    closest_obj = None
    for c in contours:
        epsilon = 0.05 * cv2.arcLength(c, True)
        if (closest_obj is None or cv2.contourArea(c) > cv2.contourArea(closest_obj)):
            appr = cv2.approxPolyDP(c, epsilon, True)
            if len(appr) == 4:
                closest_obj = c
    
    # Write largest detected red object to image
    if closest_obj is not None:
        rot_rect = cv2.minAreaRect(closest_obj)
        box = cv2.boxPoints(rot_rect)
        box = numpy.int0(box)
        cv2.drawContours(img,[box],0,(0,0,0),2)
        
    return img, closest_obj



####################################### MAIN LOGIC METHOD #######################################

def main_logic():

    # Declare algorithm variables
    path = Path()
    object_in_hand = False
    verify = False
    arrow = 0
    row = 0
    col = 0
    speed_set = 40

    # Time tracking
    base_time = time()
    dfs_time = time()
    
    test = True
    if test:
        for frame in cam.capture_continuous(rawCapture, resize=CAM_RES, format="bgr", use_video_port=True):
            # Get a snapshot from RPi camera and
            img = frame.array
            (img, closest_obj) = findObjects(img)

            # Display edited image
            cv2.imshow('Stream', img)
            cv2.waitKey(1)
            rawCapture.truncate(0)

            status = path.wallDetected(img, closest_obj)

            print(status)
            
            if status == 'grab':
                print('grabbing grabbing!!')

    # Main AI running block: Runs a continuous stream of video from RPi camera
    for frame in cam.capture_continuous(rawCapture, CAM_RES, format="bgr"):

        # Get a snapshot from RPi camera and detect if any objects are in view
        img = frame.array
        img, closest_object = findObjects(img)
            
        # Display edited image
        cv2.imshow('Stream', img)
        cv2.waitKey(1)
        rawCapture.truncate(0)

        # Create a status flag depending on distance from detected objects or walls
        status = path.wallDetected(closest_object, img)

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
                    move.move(speed_set, 'no', 'right', 0.25)
                    arrow = (arrow+1) % 4

                elif status == 'wall':
                    move.motorStop()
                    while status == 'wall':
                        move.move(speed_set, 'no', 'right', 0.25)
                        arrow = (arrow+1) % 4
                        status = path.wallDetected()

                elif status == 'grab':
                    move.motorStop()
                    grab_sequence = 0
                    base_time = path.times.pop()

                else:
                    move.move(speed_set, 'forward', 'no', 0)

            # If area has been travelled, turn right
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

    # Clean up GPIO and exit
    move.destroy()


if __name__ == '__main__':
    move.setup()
    sense.sensor_setup()
#     servoPosInit()
    main_logic()
