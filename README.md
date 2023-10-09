Engineering Documentation | Team Dominus | Explorer Robotics | Canada
====

This repository contains the engineering process including materials, software,  schematic, pictures, and descriptions of Team Dominus's self-driven vehicle model participating in the WRO Future Engineers competition in the season 2023.
----

## Team Members: 
- Aaron Xu - email: <aaronssupmail@gmail.com>
- Rudransh Srivastava - email: <rudranshsri55@gmail.com>
- Daniel Chen - email: <daniel.chen0113@gmail.com>

----

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the links to the videos where driving demonstrations exists
* `schemes` contains a schematic diagram in form of a PNG of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. 
----


## Parts List

ELEGOO UNO Project Super Starter Kit:
https://www.amazon.ca/Elegoo-Project-Starter-Tutorial-Arduino/dp/B01D8KOZF4

Traxxas LaTrax Rally Racer Kit 1/18 4WD: 
https://www.canadahobbies.ca/product/hobby-brands/latrax/traxxas-latrax-rally-118-4wd-rtr-rally-racer-orange/

Gikfun Uno R3 Case for Arduino:
https://www.amazon.ca/dp/B00Y097LY0?ref_=cm_sw_r_apin_dp_VEAKE28S3833N79SS3W7

RC Car Motor ESC Combo:
https://www.amazon.ca/Motor-Combo-Brushless-4500KV-Waterproof/dp/B08CRRQW4C/ref=sr_1_49?crid=P2A4XCRS9OK0&keywords=brushless+motor+1%2F18&qid=1681677004&s=toys&sprefix=brushless+motor+1%2F18%2Ctoys%2C118&sr=1-49

Raspberry Pi Wide Angle Camera Module: 
https://ca.robotshop.com/products/raspberry-pi-wide-angle-camera-module?gclid=CjwKCAjwue6hBhBVEiwA9YTx8LdoLgACDA32MVlrqb8tmGhgLXy8sio1_XF3TFZWEFzqf3clRY89MhoCOFoQAvD_BwE

Pisugar S Pro Portable 5000 mAh UPS Lithium Battery Power Module:
https://www.amazon.ca/Portable-Platform-Raspberry-Accessories-handhold/dp/B097RCFHD2/ref=d_pd_sbs_sccl_3_1/138-9717784-0808925?pd_rd_w=uvZjm&content-id=amzn1.sym.b35f7e0d-2e69-4a5a-a19e-76ee06774581&pf_rd_p=b35f7e0d-2e69-4a5a-a19e-76ee06774581&pf_rd_r=61YSZKJXAM0D84C39X72&pd_rd_wg=DRz71&pd_rd_r=0459cf98-0a2a-427b-995e-4519f7d4925a&pd_rd_i=B097RCFHD2&psc=1

Raspberry Pi Model 4:
https://www.amazon.ca/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27/ref=asc_df_B07TD42S27/?tag=googleshopc0c-20&linkCode=df0&hvadid=335380394635&hvpos=&hvnetw=g&hvrand=1557654368064585468&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9000759&hvtargid=pla-774661502856&psc=1

DKARDU Rocker Switch 4 Pins 2 Position ON/Off Red LED Light Illuminated DPST Switch:
https://www.amazon.ca/gp/product/B09TKMR8J4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1

CERRXIAN 90 Degree USB A to USB B Printer Cable:
https://www.amazon.com/CERRXIAN-1FT-USB-Cable/dp/B0B74F2YYZ

Hosim 2pcs 7.4V 1600mAh 25C T Connector Replacement Rechargable Battery:
https://www.amazon.ca/Hosim-2pcs-1600mAh-Connector-Li-Polymer/dp/B098QSLJKV/ref=sr_1_1?crid=3V6BDXMLEJUCW&keywords=high+c+rating+lipo&qid=1693796045&sprefix=high+c+rating+lipo%2Caps%2C105&sr=8-1

### Model 2 added parts 
Mamba Micro X2, 16.8V Waterproof ESC Motor Combo:
https://www.horizonhobby.com/product/mamba-micro-x2-16.8v-waterproof-esc-with-0808-4100kv-combo/CSE010016901.html


----


## System Design

Each part was selected based on a certain criteria:

ELEGOO UNO Project Super Starter Kit:
Additional hardware parts including wires that we could use for our robot.   

Traxxas La Trax Rally Racer Kit 1/18 4WD:
Sturdy chassis that fits the dimensions, along with being lightweight. This served as the base for our vehicle which we could build around. Moreover, it comes with a bumper which proved beneficial during test runs as it prevented damage to our vehicle. 

Gikfun Uno R3 Case for Arduino:
Case for our Arduino which we used for easy attachment onto the vehicle. Also protects the Arduino from getting damaged.

RC Car Motor ESC Combo:
Brushless motor for greater energy efficiency since there aren't any brushes creating friction and losing energy. Additionally, the brushless motor will have better speed and higher torque resulting in quicker turning capability and greater efficiency for our robot. Moreover, the brushless motor allows us to have better control of the speed of our robot over the original brushed motor that the chassis came with. 

Raspberry Pi Wide Angle Camera Module:
This is the best camera that is compatible with Raspberry Pi for us, having great colour quality and high fps which is important for our color lane detection system. It is used to detect walls and obstacles for both the open and obstacle challenges.  

Pisugar S Pro Portable 5000 mAh UPS Lithium Battery Power Module:
Battery pack for Raspberry Pi which includes a push button for the challenge. 

Raspberry Pi Model 4:
Serves as the brain of the robot. The code on the Raspberry Pi does the image processing and sends the control signals to the Arduino. Most of the coding is done on the Raspberry Pi. 

DKARDU Rocker Switch 4 Pins 2 Position ON/Off Red LED Light Illuminated DPST Switch:
Switch to turn on the Raspberry Pi and electronic speed controller with one switch.

CERRXIAN 90 Degree USB A to USB B Printer Cable:
90-degree cables to replace the old protruding wires which collided with obstacles.   

Hosim 2pcs 7.4V 1600mAh 25C T Connector Replacement Rechargable Battery:
Battery for motor to keep the vehicle in function.

## Build Process

First off, we started our process by purchasing the Traxxas LaTrax Rally ⅛ 4WD RTR Rally Racer kit. Upon its arrival, we took off the cover and decided to make modifications. The first change we made was replacing the original motor with the 2435 Brushless 4500KV Motor. This ensured that our car would be powerful and precise enough to perform in the competition. Additionally, the brushless DC motor would not deteriorate like the brushed DC motor. Throughout the process of changing the motor, we had to replace the original electronic speed controller with the new compatible electronic speed controller and remove the radio receiver. Since we needed a way to supply power to the servo, connect the signal cables from the Arduino to the servo motor and DC motor, and grounding the DC motor and servo motor, we made a plug-in chip with soldered connections which we used to connect the signal wires, power wires, and ground wires to the servo and DC motors. Next, we made many prototypes for the vehicle base, and a camera mount using cardboard and black duct tape. After coming up with the perfect design, we made it sturdy for the final product. A cut clipboard for a base to hold the car’s parts. This was suspended by four, one cm standoffs screwed to the chassis plate. We also made the camera mount out of Lego because it was lightweight and sturdy. The lego was glued together at the end to ensure stability and sturdiness. Afterwards, we screwed our camera into the mount and positioned it so it could detect as much as possible, including the ground and walls. Then we attached the Arduino and the Raspberry Pi onto the base clipboard of the robot. The last thing we had to do was connect all the wires to their respective ports and manage them properly using electrical tape and zip-ties to prevent them from protruding. This in turn helped to keep our robot compact and organized.

## Overview of Code.

To complete the open and obstacle challenges, we needed to program computer vision in order to maneuver the course. This was done on the Raspberry Pi. The Raspberry Pi then sends number commands to the Arduino, which in turn controls the DC motor speed and steering. The Arduino code is coded in c++ and receives a number from the Raspberry Pi. This number dictates what action the Arduino should perform: a signal of 1500 is a complete stop, under 1500 is forward, over 1500 is backward, and the number over 2000 is the angle at which the servo must steer. We used a variety of libraries on the Raspberry Pi. Most notably, we used OpenCV (cv2) for camera operations, Time for time-related functions, Picamera2 for controlling the Raspberry Pi camera, Serial for communication with an Arduino, Numpy for array processing, and RPi.GPIO for handling a push-button to start the car.



## Open Challenge Code

Below are the steps in order of the program logic for the Open Challenge: 

1. **Camera Setup:** Configure and start the Raspberry Pi camera, setting parameters such as resolution, and format
   
2. **Color Thresholds:** Defines upper and lower threshold values for detecting the color black, orange and blue in the camera feed, used for contour detection.
   
3. **Variables Initialization:** Initializes various variables, including ones for controlling the car's movement, counting turns, and detecting specific colored lines on the ground.

4. **Arduino Connection:** It initializes communication with the Arduino board via a serial connection.
 
5. **Button Start:** The program waits until a push-button connected to GPIO pin 5 is pressed to start the main loop.
   
6. **Main Loop:** The main loop captures and processes frames from the camera. There are six notable sections of the Main Loop: Image Processing, Contour Detection, Wall Following, Counting Turns, Sending Commands, and Termination. In the Image Processing section, the program processes the captured image to detect black contours in the left and right regions of interest (ROI), as well as colored lines (blue and orange) on the ground (bottom region of interest). In the Contour Detection section, the program uses contour detection to detect the largest black contours on the left and right sides of the image, and the blue and orange lines on the ground. In the Wall Following section, depending on the detected contours and lines, the code adjusts the car's steering angle to either follow walls or make turns. The program uses PD (Proportional-Derivative) steering control for the wall following. The proportional part of PD steering control measures the current error between the desired position and the actual position of a system. The proportional part of PD control calculates an output signal based on the magnitude of this error. The output signal is directly proportional to the error, which means that the larger the error, the larger the control signal. The derivative part of PD control takes into account the rate of change of the error. This part helps in damping oscillations due to inertia and improves the system's ability to adapt to sudden changes or disturbances. When the robot vehicle reaches a corner turn marked by the orange or blue lines on the mat, the program disables the PD control and does a sharp turn. The program knows which way to turn based on the color of the line it first sees. In the Counting Turns section, the number of turns made by the car is counted using the  detected lines. Twelve turns are counted in total, resulting in three laps. The Sending Commands section instructs the car's movement and steering by sending control commands to the Arduino. Lastly, in the Termination section, the program ends and the car stops when the total amount of turns has been completed.


## Obstacle Challenge Code

Below are the steps in order of the program logic for the Obstacle Challenge. Note that most of the steps are very similar to the Open Challenge, the main difference is the main loop. 

1. **Camera Setup:**  Configure and start the Raspberry Pi camera, setting parameters such as resolution, and format
   
2.  **Color Thresholds:** Defines upper and lower threshold values for detecting the color black, orange and blue, red, and green in the camera feed, used for contour detection.

3. **Variables Initialization:**  Initializes various variables for controlling the car's movement, direction of movement (clockwise or counterclockwise), counting turns, detecting colors, turning, and maneuvering obstacles.

4. **Arduino Connection:** It initializes communication with an Arduino board via a serial connection.

5. **Button Start:** The program waits until a push-button connected to GPIO pin 5 is pressed to start the main loop.

6. **Main Loop:** The main loop captures and processes frames from the camera.  There are eight notable sections of the Main Loop: Image Processing, Contour Detection, Counting Turns, Checking to Switch Direction, Wall Following, Maneuvering Obstacles, Sending Commands, and Termination. In the Image Processing section, similar to the open challenge, the program processes the captured image to detect black contours on the left and right regions of interests (ROI), as well as colored lines (blue and orange) on the ground (bottom region of interest). However, it also has a large region of interest in the middle for detecting green and red obstacles, and a region of interest near the top middle for detecting the wall in front. In the Contour Detection section, the program uses contour detection with thresholding to find the largest black contours (walls) on the left and right sides of the image, largest blue and orange contours (lines) on the ground, largest red and green contours (pillars) in the middle, and the largest black contour in the middle (approaching wall). In the Counting Turns section, the number of turns made by the car is counted using the detected lines. It counts twelve turns in total, resulting in three laps. In the Checking to Switch Direction section, the program checks to see if the last pillar of the second lap is red. If so, the car must turn around. This is done by observing the color of the last seen pillar when approaching the 8th turn. If this pillar is red, a predetermined set of movements (3 point turn) will be performed to turn the car 180 degrees. 1 will also be added to the amount of total turns, and to compensate, the vehicle will do one less turn in order to return to the starting section. If the pillar is green, the car will proceed in the same direction. The Wall Following the section is the same as the open challenge: the program follows the middle of the walls using Proportional-Derivative (PD) steering control. If a pillar is detected, the Maneuvering Obstacles section overwrites the commands of the Wall Following section with commands to avoid the pillar. Like the PD steering, the greater the area of the pillar, the greater the steering angle will be.  The program sends a command to steer right to avoid red pillars, and left to avoid green pillars. If the vehicle gets too close to the front wall from navigating a pillar, the program will send a command to turn sharply in a direction based on the color of the avoided pillar. The Sending Commands section and the Termination Section of the Obstacle Challenge Main Loop both work the exact same way as in the open challenge. 

## Process to build/compile/upload the code to the Raspberry Pi
Both  the open challenge and obstacle challenge were coded in Python directly on the Raspberry Pi and saved in the Raspberry Pi. For the competition, the code must run right after booting up the Raspberry Pi, without a connected monitor. We solved this issue by adding commands to run the python program in the rc.local file.

## Process to build/compile/upload the code to the Arduino
The Arduino code was coded on the Arduino IDE using a desktop computer. The code was uploaded onto the Arduino using the USB A to B connection cable. Since the Arduino memorizes the uploaded code, this process only needs to be done once. After the code has been uploaded onto the arduino, we connect it to the Raspberry Pi using the same cable. When the Raspberry Pi code runs from startup and the serial port opens, the Arduino code will compile from the beginning and continue to run until the power is shut off. 

