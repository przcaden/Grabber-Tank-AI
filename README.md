<h1 align="center">AI Grabber Tank</h1>
<p align="center">An AI-Operated Bipedal Robot on Raspberry Pi</p>

## Features
:computer: Detect specific objects (red cubes) using OpenCV <br />
:computer: Automated grabbing mechanism <br />
:computer: Pathfinding memory <br />
:computer: Back-tracking capabilities <br />

## Introduction
&emsp; This project involves the creation, design, and application of an autonomous robot using a Raspberry Pi embedded computer. The robot features path tracing capabilities to map an environment it is placed in while maneuvering around walls or obstacles in its path. The robot utilizes computer vision and OpenCV to detect red, cube-like objects in its view. It will pick up these objects with a mechanical arm on the top of its enclosure, then backtracking to its original location and placing the object back down. <br>

:bulb: Key electrical components: A Raspberry Pi camera, 2 DC motors, five servo motors, and an ultrasonic distance sensor.

<p align="center">
  <img width="460" height="300" src="https://www.settorezero.com/wordpress/contents/2020/12/rasptank_header-1-1360x765.jpg">
</p>

## :arrow_right_hook: Pathfinding / Backtracking
&emsp; The robot's pathfinding was intended to autonomously traverse an environment it is placed in. It utilizes a Depth-First Search algorithm to do so. It only explores new territory until it reaches a valid object to be picked up. To perform this accurately and to work in conjunction with the backtracking as well, a key containing therobot's current coordinates is generated every 100 milliseconds and continues to do so until the path to the object is found. <br>
&emsp; The robot's backtracking uses generated keys in order to find its way back to its origin. It uses a Breadth-First Search algorithm upon the generated keys, which finds the shortest path back to its original location. After doing so, it will place the object back down on the floor.

## :camera_flash: Computer Vision
&emsp; In order to detect the correct objects in front of it, the robot will utilize both the Raspberry Pi camera and the ultrasonic sensor. Using the camera, image
packets are interpreted via OpenCV to detect objects. The image is converted to a Hue, Saturation, Value (HSV) color space of the image to extract any red color from 
it. Then, a binary (black and white) threshold-filtered image could be generated based on the HSV image, which could be used to extract all colors within the hue range
of red. Next, compound morphology could be used on the binary image to create a clearer shape of any potential objects of the image. This step clears out any edges of
any detected polygons of the image. Below, you may see an image of a generated binary image. In this example, a 3D-printed red cube and a marker cap are in view.

<p align="center">
  <img width="460" height="300" src="https://i.imgur.com/QnK5Nos.png">
</p>

With this cleaned up threshold image, we can then detect any polygons in view and count their total number of contours. If they contain four (forming the shape of a
rectangle or square) a bounding rectangle is drawn around the image. These objects will be selected to be picked up. An example of a finished image can be seen below.

<p align="center">
  <img width="460" height="300" src="https://i.imgur.com/cN0QhAi.png">
</p>

Once the robot detects this object, it will begin to move toward it and turn such that it will be centered in the camera's view. Using the ultrasonic sensor, we can
determine how close the object is to the robot. If the robot is within sufficient distance of the object (between 6 and 9 cm) the grab sequence will be initiated.

## :bangbang: Known Issues
&emsp; Due to issues with electronic components used with the projects, testing was not able to be fully finished. As a result, movement may be jittery or get stuck.
This is due to servo and DC motors on the original prototype breaking down.
 
## :arrows_counterclockwise: Future Updates
&emsp; Since this project was done as a project for a undergraduate college-level course, time to complete this project was limited. The following would be done if the
project was taken to completion:
  - Clean up pathfinding capabilities
  - Smooth out robot turns during movement
