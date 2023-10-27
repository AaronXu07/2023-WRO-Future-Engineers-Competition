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
 

lower_black = np.array([0,0,0]) #lower threshold values for black contour detection
upper_black = np.array([135,255,50])#away upper threshold values for black contour detection
#upper_black = np.array([150,255,53]) #was [140,255,53]

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

Clockwise = False #variable true only if the direction of movement is clockwise
CounterClockwise = False #variable to true only if the direction of movement is counter clockwise

Blue_Seen = False #variable true after the blue line is seen (necessary for tracking the turn)
Orange_Seen = False #variable true after the orange line is seen (necessary for tracking the turn)

blue_line_a = 0 #variable that tracks the blue line's area
orange_line_a = 0 #variable that tracks the orange line's area

after_turn = 0 #counts how much time passed after the turn

left_x = 0
left_w = 0

right_x = 640

surround_area_L = 0
surround_area_R = 0

ratio_L = 0
ratio_R = 0
    
if __name__ == '__main__':
    
    GPIO.setwarnings(False)#setup for the push buton to start the car
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) #setup for arduino connection
    ser.flush()
    
    sleep(2.5)#delay for arduino to get ready
    #print("ready")

sendnum=1500#starts moving the car forwards (mat 1395)
sendnum = str(sendnum) #converts the number to a string so that it can be sent
ser.write((sendnum + "\n").encode('utf-8')) #sends the command to the arduino to be processed there

sleep(1)

while True: #loop ends only when button is pressed which lets the program begin 
    if GPIO.input(5) == GPIO.LOW:
        print("Button pressed")
        break

sleep(1)


sendnum=1360#starts moving the car forwards (home 1360) (mat 1376) #was 90
sendnum = str(sendnum) #converts the number to a string so that it can be sent
ser.write((sendnum + "\n").encode('utf-8')) #sends the command to the arduino to be processed there


while True: #main program loop
    
    # read image from disk
    im= picam2.capture_array()
    
    #commented code below displays regions of interest borders and is for debugging and fixing errors in the algorithm
    
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps = str(fps)
    cv2.putText(im, fps, (8, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
    
    
    #subimages for the left and right regions of interest detecting left and right walls and lines on the ground
    L_subimage = im[235:330, 20:250]#was 235:330, 20:260
    R_subimage = im[235:330, 385:620]#was 235:330, 375:620
    
    turn_subimage = im[365:410, 180:480]
    
    
    Left_points = [(20,235), (250,235), (250,330), (20,330)] #left region of interest
    Right_points = [(620,235), (385,235), (385,330), (620,330)] #right region of interest
    
    Turn_points = [(200, 365), (450, 365), (450, 410), (200, 410)]
    
    """
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
            
            L_cnt = L_contours[LeftMaxI]
            area = cv2.contourArea(L_cnt)
            
            cv2.drawContours(L_subimage, L_contours, LeftMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(L_cnt, 0.01*cv2.arcLength(L_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(L_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            
            surround_area_L = w*h
            left_x = x
            left_w = w
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
            
            R_cnt = R_contours[RightMaxI]
            area = cv2.contourArea(R_cnt)
            
            cv2.drawContours(R_subimage, R_contours, RightMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(R_cnt, 0.01*cv2.arcLength(R_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(R_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            
            surround_area_R = w*h
            right_x = x
            right_lane_a = RightMaxA #assigns the largest right contour found to right_lane_a
                    
    
    if (len(R_contours) == 0 or RightMaxA <= 70): #the right wall area will be set to 0 if no contours are found
        right_lane_a = 0  
    if (len(L_contours) == 0 or LeftMaxA <= 70): #the left wall area will be set to 0 if no contours are found
        left_lane_a = 0
      
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV) #converts the turning region of interest that looks for lines into HSV colour format
    lower_blue = np.array([100, 140, 0], np.uint8) #away (lower threshold for blue line colour)
    #lower_blue = np.array([100, 120, 0], np.uint8) #home
    upper_blue = np.array([140, 255, 255], np.uint8) #(upper threshold for blue line colour)
    maskb = cv2.inRange(turn_imgHSV, lower_blue, upper_blue) #mask to filter everything out but the blue colour of the line
    blue_contours = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours and add them all to the blue_contours array
    
    BMaxA = 0 #area of the largest blue contour found
    BMaxI = 0 #index of the largest blue contour found
    
    if (len(blue_contours) > 0): #algorithm finds the largest contour, only happens if it finds any contours
        for i in range(len(blue_contours)): #loops through all the found contours
            B_cnt = blue_contours[i]  #access the current contour
            area = cv2.contourArea(B_cnt) #gets the area of the contour
            if(area > BMaxA): #if the current contour area is larger than the previous largest area, then it becomes the largest contour
                BMaxA = area #assigns the maximum area found to the current contour if it is the largest
                BMaxI = i #assigns the index of the current contour if it is the largest
        
        if (BMaxA > 200): #only uses the largest contour area if it is large enough #was 300
            #commented code below is used to draw contours for debugging
            
            B_cnt = blue_contours[BMaxI]
            area = cv2.contourArea(B_cnt)
            #cv2.drawContours(turn_subimage, blue_contours, BMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(B_cnt, 0.01*cv2.arcLength(B_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            
            blue_line_a = BMaxA #blue_line_area is the detected blue line area

            #print("Blue Line Area = " + str(blue_line_a))        
            #print("Number of red contours found = " + str(len(red_contours)))
                
    if (len(blue_contours) == 0 or BMaxA <= 70): #if there is no blue line contours found then it assigns the area to be 0
        blue_line_a = 0
     
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV) #converts the turning region of interest that looks for lines into HSV colour format
    lower_orange = np.array([5, 60, 50], np.uint8) # was [5, 50, 50] (lower threshold for the orange line colour)
    upper_orange = np.array([18, 255, 255], np.uint8) #(upper threshold for the orange line colour)
    masko = cv2.inRange(turn_imgHSV, lower_orange, upper_orange) #mask to filter everything out but the blue colour of the line
    orange_contours = cv2.findContours(masko, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find orange contours and add them all into the orange_contours array
    
    OMaxA = 0#area of the largest orange contour found
    OMaxI = 0#index of the largest orange contour found
    if (len(orange_contours) > 0):  #algorithm finds the largest contour, only happens if it finds any contours
        for i in range(len(orange_contours)): #loops through all of the found contours
            O_cnt = orange_contours[i] #temporary variable O_cnt is set to the current contour being analyzed
            area = cv2.contourArea(O_cnt) #gets the area of the current contour
            
            if(area > OMaxA):  #if the current contour area is larger than the previous largest area, then it becomes the largest contour
                OMaxA = area #assigns the maximum area found to the current contour if it is the largest
                OMaxI = i #assigns the index of the current contour if it is the largest
        
        if (OMaxA > 200):#only uses the largest contour area if it is large enough #was 300
            #commented code below is used to draw contours for debugging
            
            O_cnt = orange_contours[OMaxI]
            area = cv2.contourArea(O_cnt)
            
            #cv2.drawContours(turn_subimage, orange_contours, OMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(O_cnt, 0.01*cv2.arcLength(O_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            
            
            orange_line_a = OMaxA #orange_line_area is the detected orange line area

            #print("Orange Line Area = " + str(orange_line_a))        
            #print("Number of orange contours found = " + str(len(orange_contours)))
                
    if (len(orange_contours) == 0 or OMaxA <= 70):  #if there is no blue line contours found then it assigns the area to be 0
        orange_line_a = 0
        
        
    #cv2.imshow("Camera", im)
    
    
    after_turn+=1

    if(blue_line_a > 200): #tracking counterclockwise movement (detect blue line for turn) #was 300
        if(Clockwise == False and CounterClockwise == False): #only sets direction if no direction has been set, if blue line is seen first, it is counterclockwise
            CounterClockwise = True #sets CounterClowckwise variable to true so the program knows which direction the car is moving in
            print("Counterclockwise")
        if(CounterClockwise): #if the car is moveing counterclockwise and sees the blue line in the corner, the program will track the turn length
            TurnFrameCount+=1
            
        Blue_Seen = True #if the blue line is seen, it seets Blue_Seen to true
        Orange_Seen = False #sets orange seen to false to stop the sharp turn for clockwise movement
    
    elif(CounterClockwise): #adding a turn count after it passes the blue line
        if(TurnFrameCount > 2 and count < 12 and after_turn > 30): #only happens if the turn has been over 2 frames to prevent bugs 
            TurnFrameCount=0 #resets the turnframecount variable
            count+=1
            after_turn = 0
            print(count)
            
    if(orange_line_a > 200):#tracking clockwise movement (detect orange line for turn) #was 300
        if(Clockwise == False and CounterClockwise == False): #only sets direction if no direction has been set, if orange line is seen first, it is clockwise
            Clockwise = True #sets Clockwise variable to true so the program knows which direction the car is moving in
            print("Clockwise")
        if(Clockwise): #if the car is moveing clockwise and sees the orange line in the corner, the program will track the turn length
            TurnFrameCount+=1
        
        Orange_Seen = True
        Blue_Seen = False
    
    elif(Clockwise):  #adding a turn count after it passes the orange line
        if(TurnFrameCount > 2 and count < 12 and after_turn > 30): #only counts a turn if the turn has been over 2 frames to prevent bugs
            TurnFrameCount=0#resets the turnframecount variable
            count+=1
            after_turn = 0
            print(count)
     
    
    if(left_lane_a > 0 and right_lane_a > 0):
        ratio_L =(surround_area_L / left_lane_a)
        ratio_R =(surround_area_R / right_lane_a)
    else:
        ratio_L = 0
        ratio_R = 0
        
    if(ratio_L > 5):
        left_lane_a = 0
        
    if(ratio_R > 5):
        right_lane_a = 0
     
     
    #WALL FOLLOWING     
    if (right_lane_a > 150 and left_lane_a > 150): #if detects a wall on both sides of the car, it should perform wall following
            
        error = left_lane_a - right_lane_a #error is the differece between the areas of the turns, this value determines which area is larger therefore which wall the robot is closer to 
        
        if (derivative == -1):#sets derivative to be used on the second loop and on because it cannot be used on the first frame as there is no rate of change
            derivative = 0
        else:
            derivative = error-prev_error #the derivative is the difference between the previous error and the current error, the derivative value is used to smoothen the wall following
        
        kp = 0.006 #kp is the value we apply to the proportional error which converts the difference into an angle #0.005
        kd = 0.0040 #kd is the value we apply to the derivative error which helps the oscillation problem with only using proportional error
        
        steering = int((kp * error) + (kd * derivative)) #the steering angle is calculated by multiplying kp by the error and kd by the derivative error
        
        if (steering > 26): #make sure steering is limited so it doesn't steer too much 
            steering = 26
        elif (steering < -26):
            steering = -26
        
        
        sendnum = angle-steering # Greater Than 2100 = to left | Less than 2100 = to the right, the number sent is the straight angle minus the steering angle       
        
        prev_error = error #previous error is set at the end of the algorithm to be used in the next frame
        
        
    if(Clockwise):                    
                
        if (right_lane_a <= 80 and left_lane_a > 80): #if detect only left wall, it should do a large adjustment to the right
            
            if(left_lane_a < 6000):
                sendnum = 2074

            elif(left_lane_a < 9000):
                sendnum = 2072
            
            elif(left_lane_a >=9000):
                sendnum = 2068
                
        elif (right_lane_a > 80 and left_lane_a <= 80): #if detect only right wall, it should do a large adjustment to the left
        
            if(right_lane_a < 6000):
                sendnum = 2126

            elif(right_lane_a < 9000):
                sendnum = 2132
            
            elif(right_lane_a >= 9000):
                sendnum = 2138      
                
    if(CounterClockwise):
        if (right_lane_a > 80 and left_lane_a <= 80): #if detect only right wall, it should do a large adjustment to the left
        
            if(right_lane_a < 6000):
                sendnum = 2126

            elif(right_lane_a < 9000):
                sendnum = 2132
            
            elif(right_lane_a >= 9000):
                sendnum = 2138
                
                
        elif (right_lane_a <= 80 and left_lane_a > 80): #if detect only left wall, it should do a large adjustment to the right
            
            if(left_lane_a < 6000):
                sendnum = 2074

            elif(left_lane_a < 9000):
                sendnum = 2072
            
            elif(left_lane_a >=9000):
                sendnum = 2068
    
    #print(ratio_L)

    if (right_x < 40 and right_x < 230-(left_x+left_w)):
        if(ratio_R < 5):
            sendnum = 2126
        
    if (230-(left_x+left_w) < 40 and 230-(left_x+left_w) < right_x):
        if(ratio_L < 5):
            sendnum = 2074
      
      
    if (CounterClockwise == True and Blue_Seen): #if needs to turn sharp left at a corner
        sendnum = 2130
               
    if (Clockwise == True and Orange_Seen): #if needs to turn sharp right at a corner
        sendnum= 2074 #was 2076
    
        
    #print("angle is: ", sendnum)
    sendnum = str(sendnum)#converts the number into a string so that it can be sent 
    ser.write((sendnum + "\n").encode('utf-8'))#sends the steering angle to the arduino
    
    
    #ENDING SECTION
    if (count == 12): #end when see the upcoming line on ground
        FinalFrame+=1 #adds 1 to how long it needs to wait until it ends
            
        if(FinalFrame > 80):  #stops car after it has run 120 frames to make sure it ends at the correct place
            sendnum = int(sendnum)
            #if(sendnum > 2092 and sendnum < 2104): #stops the car if it is straight enough
                #sleep(0.5)
            sendnum = str(1500)#stops car
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2098)#straightens wheels
            ser.write((sendnum + "\n").encode('utf-8'))
            break #breaks out of program           
    
    if cv2.waitKey(1)==ord('q'):#wait until key ‘q’ pressed
        sendnum = str(1500)#stops car
        ser.write((sendnum + "\n").encode('utf-8'))
        sendnum = str(2100)#straightens wheels
        ser.write((sendnum + "\n").encode('utf-8'))
        break
cv2.destroyAllWindows()  
