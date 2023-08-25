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
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
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

----


## System Design

Each part was selected based on a certain criteria:

ELEGOO UNO Project Super Starter Kit:
Additional hardware parts.

Traxxas La Trax Rally Racer Kit 1/18 4WD:
Sturdy chassis that fits the dimensions along with having a lightweight. 

Gikfun Uno R3 Case for Arduino:
Case for Arduino

RC Car Motor ESC Combo:
Brushless motor for greater energy efficiency, better speed and higher torque resulting in quicker turning capability and efficiency of our robot. 

Raspberry Pi Wide Angle Camera Module:
For our needs the best camera compatible with raspberry pi with great color quality and high fps which is great for our color lane detection system. 

Pisugar S Pro Portable 5000 mAh UPS Lithium Battery Power Module:
Battery Power for robot

Raspberry Pi Model 4:
To recode the robot for the task at hand. 


## Process

First off, we started our process by purchasing the Traxxas LaTrax Rally ⅛ 4WD RTR Rally Racer kit. Upon its arrival, we took off the cover, and decided to make modifications. The first change we made was replacing the original motor with the 2435 Brushless 4500KV Motor. This ensured that our car would be powerful enough to perform in the competition. Next, we made a cover for the vehicle using cardboard and black duct tape. However, we decided that it would not be sturdy enough, so we ended up using a clipboard for holding the car’s parts and a Lego camera mount. We found this to be much more stable than the previous make. After this we 

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._
