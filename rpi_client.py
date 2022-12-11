
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

    # Determine if coordinates have been visited already. Uses a sensitivity range of 5.
    def alreadyVisited(self, row, col, arrow):
        # Run check if robot is facing left or right
        if arrow == 0 or arrow == 2:
            for i in range(5):
                testkey1 = str(row) + ',' + str(col+i)
                testkey2 = str(row) + ',' + str(col-i)
                if testkey1 in self.visited or testkey2 in self.visited:
                    return True
        # Run check if robot is facing up or down
        elif arrow == 1 or arrow == 3:
            for i in range(5):
                testkey1 = str(row+i) + ',' + str(col)
                testkey2 = str(row-i) + ',' + str(col)
                if testkey1 in self.visited or testkey2 in self.visited:
                    return True
        return False

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
                if x >= ((img_x/2) - w) * 0.9 and x <= ((img_x/2) + w)* 1.1:
                    return 'grab'
                # Wall detected
                else:
                    return 'wall'
                
            # Object detected, change direction to approach object
            print('img width: ' + str(img_x))
            print('object width: ' + str(w))
            if x > (img_x/2) - w:
                return 'redirect_left'
            if x < (img_x/2) + w:
                return 'redirect_right'

        # If no object or wall detected, carry on
        return 'none'
    

# Grab the detected object in view
def grab_sequence():
    # open claw and move base
    H_sc.singleServo(15, -1, 5) # open claw
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # move base
    H_sc.singleServo(12, -1, 3)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.2)
    
    # move middle
    H_sc.singleServo(13, -1, 3)
    sleep(0.8)
    H_sc.stopWiggle()
    sleep(0.2)
    
    # move base
    H_sc.singleServo(12, -1, 3)
    sleep(0.6)
    H_sc.stopWiggle()
    sleep(0.2)

    # close claw
    H_sc.singleServo(15, 1, 5)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.8)

    # move base back up
    H_sc.singleServo(12, 1, 2)
    sleep(1.7)
    H_sc.stopWiggle()
    # move middle back down
    H_sc.singleServo(13, 1, 2)
    sleep(1)
    H_sc.stopWiggle()

# Drop the object held by the arm
def drop_sequence():
    # move base
    H_sc.singleServo(12, -1, 3)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # move middle
    H_sc.singleServo(13, -1, 3)
    sleep(0.8)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # move base
    H_sc.singleServo(12, -1, 3)
    sleep(0.6)
    H_sc.stopWiggle()
    sleep(0.3)

    # open claw (drop object)
    H_sc.singleServo(15, -1, 5)
    sleep(0.5)
    H_sc.stopWiggle()
    
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
        
    return img, closest_obj, thresh, clean



####################################### MAIN LOGIC METHOD #######################################

def main_logic():

    # Declare algorithm variables
    path = Path()
    object_in_hand = False
    arrow = 0 # indicates direction faced (0 = left, 1 = up, 2 = right, 4 = down)
    row = 0
    col = 0
    speed_set = 50

    # Time tracking
    base_time = time()
    dfs_time = time()
    
    test = True
    if test:
        for frame in cam.capture_continuous(rawCapture, resize=CAM_RES, format="bgr", use_video_port=True):
            # Get a snapshot from RPi camera and
            img = frame.array
            (img, closest_obj, thresh, clean) = findObjects(img)

            # Display edited image (disable if no display is being used)
            cv2.imshow('Stream', img)

            # Prepare next image to be fetched from the camera
            key = cv2.waitKey(1)
            rawCapture.truncate(0)

            if key is not None:
                cv2.imwrite('thresh.png', thresh)
                cv2.imwrite('clean.png', clean)
                cv2.imwrite('newimg.png', img)

            status = path.wallDetected(img, closest_obj)

            print(status)
            
            # if status == 'grab':
            #     move.motorStop()
            #     grab_sequence()
            #     sleep(2)
            #     drop_sequence()
            # else:
            #     move.move(speed_set, 'forward', 'no', 0)

    # Main AI running block: Runs a continuous stream of video from RPi camera
    for frame in cam.capture_continuous(rawCapture, CAM_RES, format="bgr"):

        # Get a snapshot from RPi camera and detect if any objects are in view
        img = frame.array
        (img, closest_object) = findObjects(img)

        # Display edited image (disable if no display is being used)
        # cv2.imshow('Stream', img)

        # Prepare next image to be fetched from the camera
        cv2.waitKey(1)
        rawCapture.truncate(0)

        # Create a status flag, depending on distance from detected objects or walls
        status = path.wallDetected(closest_object, img)

        # Generate string representation for node in path
        key = str(row) + ',' + str(col)

        # Perform decisionmaking if an object has not been picked up
        if not object_in_hand:

            # Check if node has been traversed (DFS) and robot has traveled for 0.1 seconds
            if not path.alreadyVisited(row, col) and time() > dfs_time + 0.1:
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
                        sleep(2)
                        move.motorStop()
                        arrow = (arrow+1) % 4
                        status = path.wallDetected()

                elif status == 'grab':
                    move.motorStop()
                    base_time = path.times.pop()
                    # Turn 180 degrees
                    move.move(speed_set, 'no', 'right', 0.25)
                    sleep(4)
                    move.motorStop()
                    object_in_hand = True

                else:
                    move.move(speed_set, 'forward', 'no', 0)

            # If area has been travelled, turn right
            elif path.alreadyVisited(row, col):
                move.motorStop()
                move(speed_set, 'no', 'right', 0.25)
                sleep(2)
                move.motorStop()
                arrow = (arrow+1) % 4

            # Update tracked direction faced by robot
            curDirection = path.directions[arrow]
            row += curDirection[0]
            col += curDirection[1]

        # Perform decisionmaking for backtracking if the object has been picked up
        else:
            if not path.turns:
                sleep(2)
                drop_sequence()
                break

            # if status == 'wall' or time()-base_time <= 0.1:
            #     move.motorStop()

            #     # Otherwise, proceed in backtracking path
            #     else:
            #         turn = path.turns.pop()
            #         base_time = path.times.pop()

            #         # Reverse direction of the original path, since we are moving backwards
            #         if turn == 'L':
            #             move(speed_set, 'no', 'right', 0.25)
            #         else: move(speed_set, 'no', 'left', 0.25)

            # # Continue moving forward otherwise
            # else:
            #     move(speed_set, 'forward', 'no', 0)

    # Clean up GPIO and exit
    print('Finished object retrieval')
    move.destroy()


if __name__ == '__main__':
    move.setup()
    sense.sensor_setup()
    main_logic()
