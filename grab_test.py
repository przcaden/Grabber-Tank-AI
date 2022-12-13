
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
CAM_RES = (640,480) # regular res
#CAM_RES = (1920, 1080) # screenshot res
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
        self.visited = {'0,0'}


    # Detect if an object or wall is in view and decide what action to take
    def wallDetected(self, img, closest_object):
        img_y, img_x = img.shape[:2] # get image dimensions
        dis = sense.ultra() # calculate distance in cm to nearest object
        print('Distance: ' + str(dis) + ' cm')
        if closest_object is not None:
            x, y, w, h = cv2.boundingRect(closest_object)
            
            # Obstruction in path, detect if an object is in view
            if 6 < dis < 9:
                # If object is centered, opt to grab it
                if x >= ((img_x/2) - w) * 0.9 and x <= ((img_x/2) + w)* 1.1:
                    return 'grab'
                
            # Object detected, change direction to approach object
            if x > (img_x/2) - w:
                return 'redirect_left'
            if x < (img_x/2) + w:
                return 'redirect_right'

        elif 6 < dis < 9:
            return 'wall'
        elif dis < 6:
            return 'reverse'

        # If no object or wall detected, carry on
        return 'none'
    

# Grab the detected object in view
def grab_sequence():
    # Open claw
    H_sc.singleServo(15, -1, 5) # open claw
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # Move base servo
    H_sc.singleServo(12, -1, 3)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.2)
    
    # Move middle servo
    H_sc.singleServo(13, -1, 3)
    sleep(0.8)
    H_sc.stopWiggle()
    sleep(0.2)
    
    # Move base servo
    H_sc.singleServo(12, -1, 3)
    sleep(0.6)
    H_sc.stopWiggle()
    sleep(0.2)

    # Close claw
    H_sc.singleServo(15, 1, 5)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.8)

    # Move base back up
    H_sc.singleServo(12, 1, 2)
    sleep(1.7)
    H_sc.stopWiggle()

    # Move middle back down
    H_sc.singleServo(13, 1, 2)
    sleep(1)
    H_sc.stopWiggle()


# Drop the object held by the arm
def drop_sequence():
    # Move base servo
    H_sc.singleServo(12, -1, 3)
    sleep(0.5)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # Move middle servo
    H_sc.singleServo(13, -1, 3)
    sleep(0.8)
    H_sc.stopWiggle()
    sleep(0.3)
    
    # Move base servo
    H_sc.singleServo(12, -1, 3)
    sleep(0.6)
    H_sc.stopWiggle()
    sleep(0.3)

    # Open claw (drop object)
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
        
    return img, closest_obj



####################################### MAIN LOGIC METHOD #######################################

def main_logic():
    path = Path()

    # Main AI running block: Runs a continuous stream of video from RPi camera
    for frame in cam.capture_continuous(rawCapture, resize=CAM_RES, format="bgr", use_video_port=True):

        # Get a snapshot from RPi camera and detect if any objects are in view
        img = frame.array
        (img, closest_object) = findObjects(img)

        # Display edited image (disable if no display is being used)
        cv2.imshow('Stream', img)

        # Prepare next image to be fetched from the camera
        cv2.waitKey(1)
        rawCapture.truncate(0)

        # Create a status flag, depending on distance from detected objects or walls
        status = path.wallDetected(img, closest_object)
        print('Status: ' + status)

        if status == 'grab':
            move.motorStop()
            grab_sequence()
            sleep(2)
            drop_sequence()
            break

    # Clean up GPIO and exit
    print('Finished object retrieval')
    move.destroy()


if __name__ == '__main__':
    move.setup()
    sense.sensor_setup()
    main_logic()
