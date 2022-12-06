
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
move.setup()
sense.sensor_setup()
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
CAM_RES = (320,240)
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
        img_y, img_x = img.shape[:2] # image dimensions
        dis = sense.ultra() # distance in cm

        if dis < 3 and dis > 1:
            if closest_object is not None:
                x, y, w, h = cv2.boundingRect(closest_object)

                # Object detected, change direction to approach object
                if x > (img_x/2) + w:
                    return 'redirect_left'
                if x < (img_x/2) - w:
                    return 'redirect_right'
                # If object is centered, opt to grab it
                elif x >= (img_x/2) - w and x <= (img_x/2) + w:
                    return 'grab'

            # Wall detected
            else:
                return 'wall'

        # If no object or wall detected, carry on
        return 'none'
    

# Grab the detected object in view
def grab_sequence(img):
    grab = 0
    

# Drop the object held by the arm
def drop_sequence():
    arm_lower = 0
    hand_release = 0
    servoPosInit()
    
    
# Given a snapshot, detect any red cube-like objects
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
    print(len(contours))
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
    # Reset arm to neutral position
    servoPosInit()

    # Declare algorithm variables
    path = Path()
    object_in_hand = False
    goal_finish = False
    verify = False
    arrow = 0
    row = 0
    col = 0
    speed_set = 30
    
    test = True
    if test:
        for frame in cam.capture_continuous(rawCapture, resize=CAM_RES, format="bgr", use_video_port=True):
            # Get a snapshot from RPi camera and 
            img = frame.array
            (img, closest_obj) = findObjects(img)
            status = path.wallDetected(img, closest_obj)

            print(status)
                
            # Display edited image
            cv2.imshow('Stream', img)
            cv2.waitKey(1)
            rawCapture.truncate(0)

    # Main AI running block
    base_time = time()
    dfs_time = time()
    while not verify:
        # Run continuous stream of video from RPi camera
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

                    else:
                        move(speed_set, 'forward', 'no', 0)

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
    main_logic()
    print ('Hello juicy Wrld')
