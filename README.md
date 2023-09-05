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
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains a schematic diagram in form of a PNG of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

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

----


## System Design

Each part was selected based on a certain criteria:

ELEGOO UNO Project Super Starter Kit:
Additional hardware parts including wires that we could use for our robot.   

Traxxas La Trax Rally Racer Kit 1/18 4WD:
Sturdy chassis that fits the dimensions, along with being lightweight. This served as the base for our vehicle which we could build around. Moreover, it comes with a bumper which proved beneficial during test runs as it prevented damage to our vehicle. 

Gikfun Uno R3 Case for Arduino:
Case for our Arduino which we used for easy attachment onto the vehicle. Also protects the arduino from getting damaged.

RC Car Motor ESC Combo:
Brushless motor for greater energy efficiency sincer there aren't any brushes creating friction and losing energy. Additionally, the brushless motor will have better speed and higher torque resulting in quicker turning capability and efficiency of our robot. Moreover, the brushless motor allows us to have better control of the speed of our robot over the original brushed motor that the chassis came with. 

Raspberry Pi Wide Angle Camera Module:
For our needs the best camera that is compatible with Raspberry Pi with great colour quality and high fps which is great for our color lane detection system. Used to detect walls and obstacles for both the open and obstacle challenges.  

Pisugar S Pro Portable 5000 mAh UPS Lithium Battery Power Module:
Battery pack for Raspberry Pi which includes a push button for the challenge. 

Raspberry Pi Model 4:
Serves as the brain of the robot. The code on the raspberry pi does the image processing and sends the control signals to the arduino. Most of the coding is done on the raspberry pi. 

DKARDU Rocker Switch 4 Pins 2 Position ON/Off Red LED Light Illuminated DPST Switch:
Switch to turn on the Raspberry Pi and electronic speed controller with one switch

CERRXIAN 90 Degree USB A to USB B Printer Cable:
90-degree cables to replace the old protruding wires which collided with obstacles.   

Hosim 2pcs 7.4V 1600mAh 25C T Connector Replacement Rechargable Battery:
Battery for motor to keep the vehicle in function.

## Process

First off, we started our process by purchasing the Traxxas LaTrax Rally ⅛ 4WD RTR Rally Racer kit. Upon its arrival, we took off the cover and decided to make modifications. The first change we made was replacing the original motor with the 2435 Brushless 4500KV Motor. This ensured that our car would be powerful and precise enough to perform in the competition. Additionally, the brushless DC motor would not deteriorate like the brushed DC motor. Throughout the process of changing the motor, we had to replace the original electronic speed controller with the new compatible electronic speed controller and remove the radio receiver. Since we needed a way to supply power to the servo, connect the signal cables from the Arduino to the servo motor and DC motor, and grounding the DC motor and servo motor, we made a plug-in chip with soldered connections which we used to connect the signal wires, power wires, and ground wires to the servo motor and DC motor. Next, we made many prototypes for the vehicle base, and camera mount using cardboard and black duct tape. After coming up with the perfect design, we made it sturdy for the final product. A cut clipboard for a base to hold the car’s parts. This was suspended by four, one cm standoffs screwed to the chassis plate. We also made the camera mount out of Lego because it was lightweight and sturdy. The lego was glued together at the end to ensure stability and sturdiness. Afterwards, we screwed our camera into the mount and positioned it so it could detect as much as possible, including the ground and walls. Then we attached the arduino and the raspberry pi onto the base clipboard of the robot. The last thing we had to do was connect all the wires to their respective ports and manage them properly using electrical tape and zip-ties to prevent them from protruding, this in turn helped to keep our robot compact and organized.

## Overview of Code.

To complete the open and obstacle challenges we needed to program computer vision code as well as analyze the computer vision to  maneuver the course. This was done on the raspberry pi. The raspberry pi then sends number commands to the arduino, which controls the DC motor speed and steering. The arduino code was coded in c++ and receives a number from the raspberry pi. This number dictates what action the arduino should perform: a signal of 1500 is a complete stop, under 1500 is forward, over 1500 is backward, and the number over 2000 is the angle at which the servo must steer. On the raspberry pi, we used a variety of libraries, most notably, OpenCV (cv2) for camera operations, time for time-related functions, Picamera2 for controlling the Raspberry Pi camera, serial for communication with an Arduino, numpy for array processing, and RPi.GPIO for handling a push-button to start the car.

## Open Challenge Code

Below are the steps in order of the program logic for the Open Challenge: 

1. **Camera Setup:** Configure and start the Raspberry Pi camera, setting parameters such as resolution, and format
   
2. **Color Thresholds:** Defines upper and lower threshold values for detecting the color black, orange and blue in the camera feed, used for contour detection.
   
3. **Variables Initialization:** Initializes various variables, including ones for controlling the car's movement, counting turns, and detecting specific colored lines on the ground.

4. **Arduino Connection:** It initializes communication with the Arduino board via a serial connection.
 
5. **Button Start:** The program waits until a push-button connected to GPIO pin 5 is pressed to start the main loop.
   
6. **Main Loop:** In the main loop, it captures frames from the camera and processes them. There are six main sections of the main loop: Image Processing, Contour Detection, Wall Following, Counting Turns, Sending Commands, and Termination. In the Image Processing section, the program processes the captured image to detect black contours in the left and right regions of interest (ROI), as well as colored lines (blue and orange) on the ground (bottom region of interest). In the Contour Detection section, the program uses contour detection to find the largest black contours on the left and right sides of the image. It also detects blue and orange colored lines on the ground. In the Wall Following section, depending on the detected contours and lines, the code adjusts the car's steering angle to follow walls or make turns. The program uses PD (Proportional-Derivative) steering control for the wall following. The proportional part of PD control measures the current error between the desired position and the actual position of a system. The proportional part of PD control calculates an output signal based on the magnitude of this error. The error is calculated using the difference between the areas of the walls detected. The larger the area of the wall, the closer it is to the vehicle. The output signal is directly proportional to the error, which means that the larger the error, the greater the control signal. The derivative part of PD control takes into account the rate of change of the error. This part helps in damping oscillations due to inertia and improving the system's ability to respond to sudden changes or disturbances. When the robot car reaches a corner turn, marked by the orange or blue lines on the mat, the program disables the PD control and does a sharp turn. The program knows which way it must make a sharp turn based on the coloured line it sees first. In the Counting Turns section, it counts the number of turns made by the car based on the detected lines. It counts 12 turns in total resulting in 3 laps. In the Sending Commands section, the program sends control commands to the Arduino, instructing the car's movement and steering commands. Lastly, in the Termination section, When the total number of turns has been completed the program stops the car then ends. 


## Obstacle Challenge Code

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._
