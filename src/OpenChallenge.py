import cv2
import time
from picamera2 import Picamera2
import serial
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
#importing libraries used for the program, cv2 for camera, time for waiting time, serial to communicate with arduino, numpy for array processing and RPi.GPIO for button start

#setup for camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480) #defining frame size and resolution
picam2.preview_configuration.main.format = "RGB888" 
picam2.preview_configuration.controls.FrameRate = 60 #seting frame rate
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
 

#upper_black = np.array([110,255,30])
lower_black = np.array([0,0,0]) #lower threshold values for black contour detection
#upper_black = np.array([100,255,30])#upper threshold values for black contour detection
upper_black = np.array([170,255,55])

derivative = -1 #derivative variable for smooth lane follow, will be used when the program is run
sendnum = 1500 #send num is the value sent to arduino, 1500 is stop, under 1500 is forward, above 2000 shows the angle
angle = 2098#angle for driving straight, any movements will subtract or add degrees to this angle

#colors in BGR
YELLOW = (0, 255, 255)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

# Line thickness of 4 px
thickness = 4

prev_frame_time = 0
new_frame_time = 0

count = 0#variable which counts how many turns the car has made
TurnFrameCount = 0 #variable that tracks how many frames the turn has lasted to make sure the same turn isnt counted multiple times
FinalFrame = 0 #counts how many frames has passed since the last turn to stop in the correct section

Clockwise = False
CounterClockwise = False

Blue_Seen = False
Orange_Seen = False

blue_line_a = 0
orange_line_a = 0 
if __name__ == '__main__':
    
    GPIO.setwarnings(False)#setup for the push buton to start the car
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) #setup for arduino connection
    ser.flush()
    
    sleep(8)#delay for arduino to get rady
    #print("ready")

while True: #loop ends only when button is pressed which lets the program begin 
    if GPIO.input(5) == GPIO.LOW:
        print("Button pressed")
        break

sleep(2.5)

while True: #main program loop
    
    # read image from disk
    im= picam2.capture_array()
    
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps = str(fps)
    cv2.putText(im, fps, (8, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
    
    Left_points = [(15,230), (240,230), (240,325), (15,325)] #left region of interest
    Right_points = [(620,225), (380,225), (380,320), (620,320)] #right region of interest
    
    Turn_points = [(200, 340), (450, 340), (450, 390), (200, 390)]
    
    # Using cv2.line() method to draw a yellow line with thickness of 4 px
    im = cv2.line(im, Left_points[0], Left_points[1], YELLOW, thickness)
    im = cv2.line(im, Left_points[1], Left_points[2], YELLOW, thickness)
    im = cv2.line(im, Left_points[2], Left_points[3], YELLOW, thickness)
    im = cv2.line(im, Left_points[3], Left_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Right_points[0], Right_points[1], YELLOW, thickness)
    im = cv2.line(im, Right_points[1], Right_points[2], YELLOW, thickness)
    im = cv2.line(im, Right_points[2], Right_points[3], YELLOW, thickness)
    im = cv2.line(im, Right_points[3], Right_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Turn_points[0], Turn_points[1], YELLOW, thickness)
    im = cv2.line(im, Turn_points[1], Turn_points[2], YELLOW, thickness)
    im = cv2.line(im, Turn_points[2], Turn_points[3], YELLOW, thickness)
    im = cv2.line(im, Turn_points[3], Turn_points[0], YELLOW, thickness)
    """

    #subimages for the left and right regions of interest detecting left and right walls
    L_subimage = im[230:325, 20:250] 
    R_subimage = im[225:330, 375:620]
    
    turn_subimage = im[350:400, 180:480]
    
    #converts the images into HSV colour format to setup conour detection
    L_imgHSV = cv2.cvtColor(L_subimage, cv2.COLOR_BGR2HSV)
    R_imgHSV = cv2.cvtColor(R_subimage, cv2.COLOR_BGR2HSV)
    
    #mask to search for black colour is applied to the left and right HSV subimages
    L_mask = cv2.inRange(L_imgHSV, lower_black, upper_black)
    R_mask = cv2.inRange(R_imgHSV, lower_black, upper_black)
    
    
    #left countours
    L_contours = cv2.findContours(L_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] #finds all the black contours in the left_subimage after filters applied and stores them in L_contours

    LeftMaxA = 0 #maximum contour in the array, only use the maximum value for wall follow
    LeftMaxI = 0 #index of the maximum contour
    
    if (len(L_contours) > 0): #algorithm to find the maximum contour area and index
        for i in range(len(L_contours)): #goes through the array of contours
            L_cnt = L_contours[i] 
            area = cv2.contourArea(L_cnt) #finds the area of the current contour
            if(area > LeftMaxA): #if the area is bigger than the current max area, then it becomes the largest contour
                LeftMaxA = area
                LeftMaxI = i     
        
        if (LeftMaxA > 20): #if the contour is greater than 20, prevents tiny specs from detecting as the wall
            """
            L_cnt = L_contours[LeftMaxI]
            area = cv2.contourArea(L_cnt)
            
            cv2.drawContours(L_subimage, L_contours, LeftMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(L_cnt, 0.01*cv2.arcLength(L_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(L_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            """
            left_lane_a = LeftMaxA #assigns the maximum left contour area to left_lane_a
  
    
    #right countours
    R_contours = cv2.findContours(R_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]#finds all the black contours in the right_subimage after filters applied and stores them in R_contours
    
    #Right countour
    RightMaxA = 0 #stores the contour area for the largest contour found
    RightMaxI = 0 #stores the index of the largest contour found
    
    if (len(R_contours) > 0):#algorithm for detecting the largest contour
        for i in range(len(R_contours)): #cycles through all of the contours found
            R_cnt = R_contours[i] 
            area = cv2.contourArea(R_cnt) #finds the area of the current contour
            if(area > RightMaxA):#if the area of the current contour is bigger than the previous largest area, then it becomes the largest contour
                RightMaxA = area #assigns values
                RightMaxI = i 
        
        if (RightMaxA > 20): #if the contour is above area of 20 it is used, to prevent tiny specs of black from detecting as a wall
            """
            R_cnt = R_contours[RightMaxI]
            area = cv2.contourArea(R_cnt)
            
            cv2.drawContours(R_subimage, R_contours, RightMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(R_cnt, 0.01*cv2.arcLength(R_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(R_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            """
            right_lane_a = RightMaxA #assigns the largest right contour found to right_lane_a
                    
            #print("Number of Right Contours found = " + str(len(R_contours)))
    
    if (len(R_contours) == 0 or RightMaxA <= 70): #the right wall area will be set to 0 if no contours are found
        right_lane_a = 0  
    if (len(L_contours) == 0 or LeftMaxA <= 70): #the left wall area will be set to 0 if no contours are found
        left_lane_a = 0
      
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 140, 0], np.uint8) #away
    #lower_blue = np.array([100, 120, 0], np.uint8) #home
    upper_blue = np.array([140, 255, 255], np.uint8)
    maskb = cv2.inRange(turn_imgHSV, lower_blue, upper_blue)
    blue_contours = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours
    
    BMaxA = 0
    BMaxI = 0
    if (len(blue_contours) > 0):
        for i in range(len(blue_contours)):
            B_cnt = blue_contours[i]
            area = cv2.contourArea(B_cnt)
            if(area > BMaxA):
                BMaxA = area
                BMaxI = i 
        
        if (BMaxA > 300):
            """
            B_cnt = blue_contours[BMaxI]
            area = cv2.contourArea(B_cnt)
            #cv2.drawContours(turn_subimage, blue_contours, BMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(B_cnt, 0.01*cv2.arcLength(B_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            """
            blue_line_a = BMaxA

            #print("Blue Line Area = " + str(blue_line_a))        
            #print("Number of red contours found = " + str(len(red_contours)))
                
    if (len(blue_contours) == 0 or BMaxA <= 70):
        blue_line_a = 0
     
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 60, 50], np.uint8) # was [5, 50, 50]
    upper_orange = np.array([18, 255, 255], np.uint8)
    masko = cv2.inRange(turn_imgHSV, lower_orange, upper_orange)
    orange_contours = cv2.findContours(masko, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours
    
    OMaxA = 0
    OMaxI = 0
    if (len(orange_contours) > 0):
        for i in range(len(orange_contours)):
            O_cnt = orange_contours[i]
            area = cv2.contourArea(O_cnt)
            if(area > OMaxA):
                OMaxA = area
                OMaxI = i 
        
        if (OMaxA > 300):
            """
            O_cnt = orange_contours[OMaxI]
            area = cv2.contourArea(O_cnt)
            
            #cv2.drawContours(turn_subimage, orange_contours, OMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(O_cnt, 0.01*cv2.arcLength(O_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            """
            orange_line_a = OMaxA

            #print("Orange Line Area = " + str(orange_line_a))        
            #print("Number of orange contours found = " + str(len(orange_contours)))
                
    if (len(orange_contours) == 0 or OMaxA <= 70):
        orange_line_a = 0
        
        
    #cv2.imshow("Camera", im)


    sendnum=1375#starts moving the car forwards (home 1360) (mat 1370)
    sendnum = str(sendnum) #converts the number to a string
    ser.write((sendnum + "\n").encode('utf-8')) #sends the sendnum to the arduino to be processed there
    
    
    if(blue_line_a > 300): #counting counterclockwise movement (detect blue line for turn)
        if(Clockwise == False and CounterClockwise == False):
            CounterClockwise = True
            print("Counterclockwise")
        if(CounterClockwise):
            TurnFrameCount+=1
            
        Blue_Seen = True
        Orange_Seen = False
            
    elif(orange_line_a > 300):#counting cclockwise movement (detect orange line for turn)
        if(Clockwise == False and CounterClockwise == False):
            Clockwise = True
            print("Clockwise")
        if(Clockwise):
            TurnFrameCount+=1
        
        Orange_Seen = True
        Blue_Seen = False
    
    else: #adding count on turn
        if(TurnFrameCount > 2 and count < 12): 
            TurnFrameCount=0
            count+=1
            print(count)
            
    if (right_lane_a > 150 and left_lane_a > 150): #if detects a wall on both sides of the car, it should perform wall following
        
        if (right_lane_a > 775 and left_lane_a > 775): #only add turn count when the walls are big enough, meaning the turn has entirely finished before we count the turn
            if (TurnFrameCount > 15): #second precaution requiring the turn to last long enough, to prevent misscounting
                if (count < 12): #stops counting turns when it has done enough turns
                    count+=1
                    print(count)
                    TurnFrameCount = 0
            
        error = left_lane_a - right_lane_a #error is the differece between the areas of the turns, this value determines which area is larger therefore which wall the robot is closer to 
        
        if (derivative == -1):#sets derivative to be used on the second loop and on because it cannot be used on the first frame as there is no rate of change
            derivative = 0
        else:
            derivative = error-prev_error #the derivative is the difference between the previous error and the current error, the derivative value is used to smoothen the wall following
        
        kp = 0.005 #kp is the value we apply to the proportional error which converts the difference into an angle
        kd = 0.0040 #kd is the value we apply to the derivative error which helps the oscillation problem with only using proportional error
        
        steering = int((kp * error) + (kd * derivative)) #the steering angle is calculated by multiplying kp by the error and kd by the derivative error
        
        if (steering > 26): #make sure steering is limited so it doesn't steer too much 
            steering = 26
        elif (steering < -26):
            steering = -26
        
        
        sendnum = angle-steering # Greater Than 2100 = to left | Less than 2100 = to the right, the number sent is the straight angle minus the steering angle       
        
        prev_error = error #previous error is set at the end of the algorithm to be used in the next frame
        
    elif (right_lane_a > 80 and left_lane_a <= 80): #if detect only right wall, it should 
        
        if(right_lane_a < 6000):
            sendnum = 2126

        elif(right_lane_a < 9000):
            sendnum = 2132
        
        elif(right_lane_a >= 9000):
            sendnum = 2138
            
            
    elif (right_lane_a <= 80 and left_lane_a > 80): #if detect only left wall
        
        if(left_lane_a < 6000):
            sendnum = 2074

        elif(left_lane_a < 9000):
            sendnum = 2072
        
        elif(left_lane_a >=9000):
            sendnum = 2068


    if (CounterClockwise == True and Blue_Seen): #if needs to turn sharp left
        sendnum = 2132
               
    if (Clockwise == True and Orange_Seen): #if needs to turn sharp right  
        sendnum= 2074
    
    #print("angle is: ", sendnum)
    sendnum = str(sendnum)#converts the number into a string so that it can be sent 
    ser.write((sendnum + "\n").encode('utf-8'))#sends the steering angle to the arduino
    
    
    #ENDING SECTION
    if (count == 12): #end when see the upcoming line on ground
        FinalFrame+=1
            
        if(FinalFrame > 85): 
            sendnum = str(1500)#stops car
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2098)
            ser.write((sendnum + "\n").encode('utf-8'))
            break #breaks out of program           
    
    if cv2.waitKey(1)==ord('q'):#wait until key ‘q’ pressed
        sendnum = str(1500)#stops car
        ser.write((sendnum + "\n").encode('utf-8'))
        sendnum = str(2100)
        ser.write((sendnum + "\n").encode('utf-8'))
        break
cv2.destroyAllWindows()  