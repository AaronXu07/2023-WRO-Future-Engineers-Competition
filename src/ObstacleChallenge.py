import cv2
import time
from picamera2 import Picamera2
import serial
import readchar
from readchar import readkey, key
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

derivative = -1 #derivative variable for smooth lane follow, will be used when the program is run
sendnum = 1500 #send num is the value sent to arduino, 1500 is stop, under 1500 is forward, above 2000 shows the angle
angle = 2098#angle for driving straight, any movements will subtract or add degrees to this angle

#colours in BGR
YELLOW = (0, 255, 255)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (245, 206, 135)
PINK = (121, 28, 227)
NAVY = (68, 42, 32)
PURPLE = (128, 0, 128)

# Line thickness of 3 px (just for display debugging)
thickness = 3

prev_frame_time = 0
new_frame_time = 0

count = 0 #variable which counts how many turns the car has made
TurnFrameCount = 0#variable that tracks how many frames the turn has lasted to make sure the same turn isnt counted multiple times
FinalFrame = 0#counts how many frames has passed since the last turn to stop in the correct section

Clockwise = False #variable true only if the direction of movement is clockwise
CounterClockwise = False #variable to true only if the direction of movement is counter clockwise

Blue_Seen = False #variable true after the blue line is seen (necessary for tracking the turn)
Orange_Seen = False #variable true after the orange line is seen (necessary for tracking the turn)
        
green_pillar_area = 0 #variable that tracks the area of the largest green pillar found in the middle region of interest
red_pillar_area = 0#variable that tracks the area of the largest red pillar found in the middle region of interest

blue_line_a = 0#variable that tracks the blue line's area
orange_line_a = 0#variable that tracks the orange line's area

Back_wall_a = 0 #variable that tracks the area of the back wall that the car is approaching head on

Last_Pillar = "" #string variable that tracks the last seen pillar colour (mainly used for turning around)

if_turnaround = True #if the program should proceed with switching the direction, (set to false if the last pillar of the 2nd lap is green)

pillar_frames = 0 #frame counter to prevent detection of the first pillar of the next straight section during the turn

after_turn = 0 #counts how much time passed after the turn

Left_points = [(5,265), (185,265), (185,390), (5,390)] #left region of interest
Right_points = [(635,265), (460,265), (460,390), (635,390)] #right region of interest
Mid_points = [(130, 218), (515, 218), (515, 420), (130, 420)]
Turn_points = [(185, 285), (465, 285), (465, 310), (185, 310)]
back_points = [(165, 225), (500, 225), (500, 285), (165, 285)]


left_x = 0
left_w = 0

right_x = 640

surround_area_L = 0
surround_area_R = 0

red_wait = 10 #was 11/12

final_turnaround = 0

time_last = 0

if __name__ == '__main__':
    
    GPIO.setwarnings(False)#setup for the push buton to start the car
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)#setup for arduino connection
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


sendnum=1380#starts moving the car forwards (mat 1380)
sendnum = str(sendnum) #converts the number to a string so that it can be sent
ser.write((sendnum + "\n").encode('utf-8')) #sends the command to the arduino to be processed there


#MAIN LOOP
while True:
    # read image from disk
    im= picam2.capture_array()
    
    ratio_L = 0
    ratio_R = 0


    red_rect = 0
    ratio_red = 0

    O_rect = 0
    ratio_O = 0

    B_rect = 0
    ratio_B = 0

    L_subimage = im[265:390, 0:185] #Subimage that is analyzed to find the left wall #was 265:390, 5:185
    R_subimage = im[265:390, 460:640] #Subimage that is analyzed to find the right wall #was 265:390, 460:635
    
    M_subimage = im[217:422, 134:512]  #Subimage that is analyzed to find the pillars #was im[218:422, 132:512]
    turn_subimage = im[285:315, 185:465] #Subimage that is analyzed to find the lines on the floor (was im[285:313, 185:465])
    back_wall = im[220:280, 165:500] #Subimage that is analyzed to find the approaching wall in the front
    
    #commented code below displays regions of interest borders and is for debugging and fixing errors in the algorithm
    # Using cv2.line() method to draw a yellow line with thickness of 4 px
    """
    im = cv2.line(im, Left_points[0], Left_points[1], YELLOW, thickness)
    im = cv2.line(im, Left_points[1], Left_points[2], YELLOW, thickness)
    im = cv2.line(im, Left_points[2], Left_points[3], YELLOW, thickness)
    im = cv2.line(im, Left_points[3], Left_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Right_points[0], Right_points[1], YELLOW, thickness)
    im = cv2.line(im, Right_points[1], Right_points[2], YELLOW, thickness)
    im = cv2.line(im, Right_points[2], Right_points[3], YELLOW, thickness)
    im = cv2.line(im, Right_points[3], Right_points[0], YELLOW, thickness)
    """
    """
    im = cv2.line(im, Mid_points[0], Mid_points[1], PURPLE, thickness)
    im = cv2.line(im, Mid_points[1], Mid_points[2], PURPLE, thickness)
    im = cv2.line(im, Mid_points[2], Mid_points[3], PURPLE, thickness)
    im = cv2.line(im, Mid_points[3], Mid_points[0], PURPLE, thickness)
    
    im = cv2.line(im, Turn_points[0], Turn_points[1], YELLOW, thickness)
    im = cv2.line(im, Turn_points[1], Turn_points[2], YELLOW, thickness)
    im = cv2.line(im, Turn_points[2], Turn_points[3], YELLOW, thickness)
    im = cv2.line(im, Turn_points[3], Turn_points[0], YELLOW, thickness)
    """
    """
    im = cv2.line(im, back_points[0], back_points[1], YELLOW, thickness)
    im = cv2.line(im, back_points[1], back_points[2], YELLOW, thickness)
    im = cv2.line(im, back_points[2], back_points[3], YELLOW, thickness)
    im = cv2.line(im, back_points[3], back_points[0], YELLOW, thickness)
    """
    
    lower_black = np.array([0,0,0])#lower threshold values for black contour detection

    upper_black = np.array([130,255,35])#upper threshold values for black contour detection
    #was[140,255,53]

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
    
    if (len(L_contours) > 0): #algorithm to find the maximum left contour area and index
        for i in range(len(L_contours)): #goes through the array of contours
            L_cnt = L_contours[i]
            area = cv2.contourArea(L_cnt) #finds the area of the current contour
            if(area > LeftMaxA): #if the area is bigger than the current max area, then it becomes the largest contour
                LeftMaxA = area
                LeftMaxI = i     
        
        if (LeftMaxA > 20):#if the contour is greater than 20, prevents tiny specs from detecting as the wall
            
            L_cnt = L_contours[LeftMaxI]
            area = cv2.contourArea(L_cnt) 
            cv2.drawContours(L_subimage, L_contours, LeftMaxI, (255,255,255), 2)
            approx=cv2.approxPolyDP(L_cnt, 0.01*cv2.arcLength(L_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            ly=y
            #cv2.rectangle(L_subimage,(x,y),(x+w,y+h),(255,255,255),2)
              
            surround_area_L = w*h
            left_x = x
            left_w = w
            left_lane_a = LeftMaxA #assigns the maximum left contour area to left_lane_a
            
    
    #right countours
    R_contours = cv2.findContours(R_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    #Right countour
    RightMaxA = 0 #stores the contour area for the largest contour found
    RightMaxI = 0 #stores the index of the largest contour found
    
    if (len(R_contours) > 0): #algorithm to find the maximum right contour area and index
        for i in range(len(R_contours)):#cycles through all of the right contours found
            R_cnt = R_contours[i]
            area = cv2.contourArea(R_cnt)#finds the area of the current contour
            if(area > RightMaxA): #if the area of the current contour is bigger than the previous largest area, then it becomes the largest contour
                RightMaxA = area #assigns values
                RightMaxI = i 
        
        if (RightMaxA > 20):#if the contour is above area of 20 it is used, to prevent tiny specs of black from detecting as a wall
            
            R_cnt = R_contours[RightMaxI]
            area = cv2.contourArea(R_cnt)
            cv2.drawContours(R_subimage, R_contours, RightMaxI, (255,255,255), 2)
            approx=cv2.approxPolyDP(R_cnt, 0.01*cv2.arcLength(R_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #cv2.rectangle(R_subimage,(x,y),(x+w,y+h),(255,255,255),2)       
            
            surround_area_R = w*h
            right_x = x
            right_lane_a = RightMaxA #assigns the largest right contour found to right_lane_a
            
    
    if (len(R_contours) == 0 or RightMaxA <= 70):#the right wall area will be set to 0 if no contours are found in the right ROI
        right_lane_a = 0  
    if (len(L_contours) == 0 or LeftMaxA <= 70): #the left wall area will be set to 0 if no contours are found in the left ROI
        left_lane_a = 0
        
    
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)#converts the turning region of interest that looks for lines into HSV colour format
    lower_blue = np.array([100, 130, 36], np.uint8) #home was 100, 130, 40
    upper_blue = np.array([140, 255, 255], np.uint8) #(upper threshold for blue line colour)
    maskb = cv2.inRange(turn_imgHSV, lower_blue, upper_blue) #mask to filter everything out but the blue colour of the line
    blue_contours = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours and add them all to the blue_contours array
    
    BMaxA = 0 #area of the largest blue contour found
    BMaxI = 0 #index of the largest blue contour found
    
    #Section finds largest blue contour takes note of the area (for counting turns, and direction)
    if (len(blue_contours) > 0):#algorithm finds the largest blue contour, only happens if it finds any contours
        for i in range(len(blue_contours)):
            B_cnt = blue_contours[i]
            area = cv2.contourArea(B_cnt)
            if(area > BMaxA):
                BMaxA = area
                BMaxI = i 
        
        if (BMaxA > 100): #was 300
            
            B_cnt = blue_contours[BMaxI]
            area = cv2.contourArea(B_cnt)
            #cv2.drawContours(turn_subimage, blue_contours, BMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(B_cnt, 0.01*cv2.arcLength(B_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            
            blue_line_a = BMaxA
            B_rect = w*h
                
    if (len(blue_contours) == 0 or BMaxA <= 70):
        blue_line_a = 0
        
    if(blue_line_a > 0):
        ratio_B = B_rect/blue_line_a
        #print(ratio_B)
    if(ratio_B<1.7): #was 2.2
        blue_line_a=0  
    
    #setup for finding orange contours with thresholding
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([9, 70, 50], np.uint8) # was [9, 75, 50] home
    upper_orange = np.array([18, 255, 255], np.uint8) #was 18, 255, 255
    masko = cv2.inRange(turn_imgHSV, lower_orange, upper_orange)
    orange_contours = cv2.findContours(masko, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours
    
    #Section finds largest orange contour takes note of the area (for counting turns, and direction)
    OMaxA = 0
    OMaxI = 0
    if (len(orange_contours) > 0):
        for i in range(len(orange_contours)):
            O_cnt = orange_contours[i]
            area = cv2.contourArea(O_cnt)
            if(area > OMaxA):
                OMaxA = area
                OMaxI = i 
        
        if (OMaxA > 100): #was 300
            
            O_cnt = orange_contours[OMaxI]
            area = cv2.contourArea(O_cnt)
            #cv2.drawContours(turn_subimage, orange_contours, OMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(O_cnt, 0.01*cv2.arcLength(O_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)
            
            orange_line_a = OMaxA
            O_rect = w*h
        
                
    if (len(orange_contours) == 0 or OMaxA <= 70):
        orange_line_a = 0
    
    if(orange_line_a >0):
        ratio_O = O_rect/orange_line_a
        #print(ratio_O)
    if(ratio_O<2.5): #was 2.7/2.6
        orange_line_a=0
        
    #section below finds the red and green pillars and takes note of their area (for manueuvering the obstacles)    
    M_imgHSV = cv2.cvtColor(M_subimage, cv2.COLOR_BGR2HSV) 
    
    """
    #lower mask (0-10)
    lower_red = np.array([0, 100, 20]) 
    upper_red = np.array([0, 255, 255]) #was ([3, 255, 255])
    mask0 = cv2.inRange(M_imgHSV, lower_red, upper_red)
    """
    
    #upper mask (170-180)
    lower_red = np.array([172, 110, 35])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(M_imgHSV, lower_red, upper_red)      
    #raw_mask = mask0 | mask1
    
    red_contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find red contours
    
    
    #lower_green = np.array([50,115,27])
    #upper_green = np.array([100,255,255])
    lower_green = np.array([50,110,20]) #was 120
    upper_green = np.array([95,255,255])
    mask = cv2.inRange(M_imgHSV, lower_green, upper_green)
    
    green_contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] #find green contours
    
    
    RedMaxA = 0
    RedMaxI = 0
    if (len(red_contours) > 0):
        for i in range(len(red_contours)):
            Red_cnt = red_contours[i]
            area = cv2.contourArea(Red_cnt)
            if(area > RedMaxA):
                RedMaxA = area
                RedMaxI = i 
        
        if (RedMaxA > 70):
            
            Red_cnt = red_contours[RedMaxI]
            area = cv2.contourArea(Red_cnt)
            cv2.drawContours(M_subimage, red_contours, RedMaxI, (235, 206, 135), 2)
            approx=cv2.approxPolyDP(Red_cnt, 0.01*cv2.arcLength(Red_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #cv2.rectangle(M_subimage,(x,y),(x+w,y+h),(235, 206, 135),2)
            
            red_pillar_area = RedMaxA
            red_y = y
            red_x = x
            
            red_rect = w*h
    
    GreenMaxA = 0
    GreenMaxI = 0
    if (len(green_contours) > 0):
        for i in range(len(green_contours)):
            G_cnt = green_contours[i]
            area = cv2.contourArea(G_cnt)
            if(area > GreenMaxA):
                GreenMaxA = area
                GreenMaxI = i 
        
        if (GreenMaxA > 70):
            
            G_cnt = green_contours[GreenMaxI]
            area = cv2.contourArea(G_cnt)
            cv2.drawContours(M_subimage, green_contours, GreenMaxI, (235, 206, 135), 2)
            approx=cv2.approxPolyDP(G_cnt, 0.01*cv2.arcLength(G_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #cv2.rectangle(M_subimage,(x,y),(x+w,y+h),(235, 206, 135),2)
            
            green_pillar_area = GreenMaxA
            green_y = y
            green_x = x
           
            
    if (len(red_contours) == 0 or RedMaxA <= 70):
        red_pillar_area = 0
    if (len(green_contours) == 0 or GreenMaxA <= 70):
        green_pillar_area = 0
    
    
    #Section below finds the contour area of the back wall (this is for avoiding obstacled avoided while going head first into the outer wall
    Back_imgHSV = cv2.cvtColor(back_wall, cv2.COLOR_BGR2HSV)
    Back_mask = cv2.inRange(Back_imgHSV, lower_black, upper_black)
    Back_contours = cv2.findContours(Back_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    #Back countour
    BackMaxA = 0
    BackMaxI = 0 
    if (len(Back_contours) > 0):
        for i in range(len(Back_contours)):
            Back_cnt = Back_contours[i]
            area = cv2.contourArea(Back_cnt)
            if(area > BackMaxA):
                BackMaxA = area
                BackMaxI = i 
        
        if (BackMaxA > 200):
            """
            Back_cnt = Back_contours[BackMaxI]
            area = cv2.contourArea(Back_cnt)
            cv2.drawContours(back_wall, Back_contours, BackMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(Back_cnt, 0.01*cv2.arcLength(Back_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(back_wall,(x,y),(x+w,y+h),(255,255,255),2)
            """
            Back_wall_a = BackMaxA
            
    if (len(Back_contours) == 0 or BackMaxA <= 200):
        Back_wall_a = 0
    
    #print("Back Wall Area", Back_wall_a)
    
    #cv2.imshow("Camera", im)
    
    pillar_frames+=1
    after_turn+=1
    #print(Last_Pillar)
    #print(pillar_frames)
    
    if(green_pillar_area <= 150 and red_pillar_area <=150):
        time_last +=1
    elif(Clockwise and Orange_Seen == False):
        if(green_pillar_area >= 500 or red_pillar_area >=500):
            time_last = 0
    elif(CounterClockwise and Blue_Seen == False):
        if(green_pillar_area >= 500 or red_pillar_area >=500):
            time_last = 0
            
            
    if(blue_line_a > 120): #counting counterclockwise movement (detect blue line for turns) #was 300
        if(Clockwise == False and CounterClockwise == False):
            CounterClockwise = True
            print("Counterclockwise")
        
        Blue_Seen = True
        Orange_Seen = False
        
        if(CounterClockwise):
            TurnFrameCount+=1
        
            if(time_last < 50):
                pillar_frames = 0
            
    
    elif(CounterClockwise): #adding count on turn
        if(TurnFrameCount > 3 and count < 12): 
            TurnFrameCount = 0
            after_turn = 0
            count+=1
            print(count)
            
            
    if(orange_line_a > 140):#counting clockwise movement (detect orange line for turns) #was 150
        if(Clockwise == False and CounterClockwise == False):
            Clockwise = True
            print("Clockwise")
        
        Orange_Seen = True
        Blue_Seen = False
        
        if(Clockwise):
            TurnFrameCount+=1
            
            if(time_last < 40):
                pillar_frames = 0
            
        
    elif(Clockwise): #adding count on turn
        if(TurnFrameCount > 4 and count < 12): 
            TurnFrameCount = 0
            after_turn = 0
            count+=1
            print(count)
    
    if(count > 0 and after_turn < 135): #was 140
        TurnFrameCount = 0
    
    
    #TURN AROUND CHECK
    if(count == 8 and if_turnaround and after_turn > 110): #was 120
        if(Last_Pillar == "red"):
            if_turnaround = False
            final_turnaround+=1
        else:
            if_turnaround = False
        
    #Section below runs if the car needs to switch drection, (if it's the end of the 2nd lap and the last pillar seen was red)
    if(final_turnaround > 0):
        after_turn = 0
        
        final_turnaround+=1
        
        
        if(Clockwise and final_turnaround > 70): #was 60
            Clockwise = False
            CounterClockwise = True
            
            sendnum = str(1380) #was all 1384
            ser.write((sendnum + "\n").encode('utf-8'))    
            
            if(green_pillar_area < 150):
                sendnum = str(2090)
                ser.write((sendnum + "\n").encode('utf-8'))
                sleep(0.1) #was 0.6/0.5
                
            sendnum = str(2138) 
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.8) #was 2
            
            sendnum = str(1586) #was 1585 / 1580
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2060)
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.8) #was 1.4
            
            sendnum = str(1500) #was 1585 / 1580
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.8)
            
            sendnum = str(1380) #was 84
            ser.write((sendnum + "\n").encode('utf-8'))
            
            sendnum = str(2080)
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.1) # was 1
             
            sendnum = str(2140) #was 2138
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(1) #was 1.3
            
            TurnFrameCount = 0
            pillar_frames = 0
            count+=1
            print(count)
            if_turnaround = False
            final_turnaround = -1
            after_turn = -40
            
            sendnum=1380#starts moving the car forwards (mat 1395)
            sendnum = str(sendnum)
            ser.write((sendnum + "\n").encode('utf-8'))
            
        elif(CounterClockwise and final_turnaround > 70): #was 60
            Clockwise = True
            CounterClockwise = False
            
            sendnum = str(1380) #was 1384
            ser.write((sendnum + "\n").encode('utf-8'))
            
            if(green_pillar_area < 150):
                sendnum = str(2090)
                ser.write((sendnum + "\n").encode('utf-8'))
                sleep(0.2) #was 0.6/0.5
            
            
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2140) #was 2138
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.7) #was 1.1
            
            sendnum = str(1586) #was 1572 / 1585
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2060)
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.9) #was 1.5
            
            sendnum = str(1500) #was 1585 / 1580
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.8)
            
            
            sendnum = str(1380) #was all 1395
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2080) #was 2120
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.1) #was 0.5
            
            
            sendnum = str(2142) #was 2138-2140
            ser.write((sendnum + "\n").encode('utf-8'))
            sleep(0.9) #was 1.1
        
            TurnFrameCount = 0
            pillar_frames = 0
            count+=1
            print(count)
            if_turnaround = False
            final_turnaround = -1
            after_turn = -40
            
            sendnum=1380#starts moving the car forwards (mat 1395)
            sendnum = str(sendnum)
            ser.write((sendnum + "\n").encode('utf-8'))
         
     
    if(left_lane_a > 0 and right_lane_a > 0):
        ratio_L =(surround_area_L / left_lane_a)
        ratio_R =(surround_area_R / right_lane_a)
    else:
        ratio_L = 0
        ratio_R = 0
        
    if(ratio_L > 3): #was 5
        left_lane_a = 0
        
    if(ratio_R > 3): #was 5
        right_lane_a = 0
    
    
    #WALL FOLLOWING       
    if (right_lane_a >= 0 and left_lane_a >= 0): #wall following only
        
        error = left_lane_a - right_lane_a 
        
        if (derivative == -1):
            derivative = 0
        else:
            derivative = error-prev_error
        
        kp = 0.008
        kd = 0.003
        
        steering = int((kp * error) + (kd * derivative))
        
        if (steering > 30): #make sure steering is limited so it doesn't steer too much 
            steering = 30
        elif (steering < -30):
            steering = -30
        
        #sharp steering if wall is not detected on one or both sides
        if(Clockwise == False and CounterClockwise == False):
            if(Last_Pillar == "green"):
                steering = 40 #was all 38
            elif(Last_Pillar == "red"):
                steering = -40
            
        if(Clockwise):
            if (right_lane_a <= 80): 
                steering = 40
            elif (left_lane_a <= 80):
                steering = -40
        
        if(CounterClockwise):
            if (left_lane_a <= 80):
                steering = -44 #was -44
            elif (right_lane_a <= 80):  
                steering = 38 # was 40
        
        sendnum = angle-steering # Greater Than 2100 = to left | Less than 2100 = to the right 
        
        prev_error = error
    
    
    #BACK WALL DETECTION (turn sharp one way if the approaching wall is very close after passing an obstacle to not crash into the wall
    if(Back_wall_a > 9800): #was 9700
        #print(Back_wall_a)
        
        if(Clockwise and Last_Pillar == "green"):
            sendnum = 2060
        
        if(CounterClockwise and Last_Pillar == "red"):
            sendnum = 2137 #was 2136
    
    
    if(red_pillar_area>0):
        ratio_red = red_rect/red_pillar_area
        #print(ratio_red)
        
    if(ratio_red>2.4):
        red_pillar_area = 0
     
    if(CounterClockwise):
        red_wait = 8
    
    if(Clockwise):
        red_wait = 10 #was 11 / 12
        
        
        
    #RED PILLAR DETECTION (and maneuvering for red pillars)
    if (red_pillar_area >= 150 and pillar_frames > red_wait): #movement for detection of red pillar #was 150 | 10/12/13
        #print("red pillar area: ", red_pillar_area)
        if(green_pillar_area >= 150):
            if(red_y > green_y):
                if(red_pillar_area <= 750):
                    sendnum = 2072 #was 72
                elif(red_pillar_area <=1500):
                    sendnum = 2068 #was 72
                elif(red_pillar_area <=2000):
                    sendnum = 2066 #was 70
                elif(red_pillar_area <=3000):
                    sendnum = 2062 #was 66
                elif(red_pillar_area <=4000):
                    sendnum = 2058 #was 62
                else:
                    sendnum = 2052 #was 58  
                
                if(pillar_frames > 10 and red_pillar_area>400):
                    Last_Pillar = "red"
                 
        else:
            if(red_pillar_area <= 750):
                sendnum = 2072 #was 72
            elif(red_pillar_area <=1500):
                sendnum = 2068 #was 72
            elif(red_pillar_area <=2000):
                sendnum = 2066 #was 70
            elif(red_pillar_area <=3000):
                sendnum = 2062 #was 66
            elif(red_pillar_area <=4000):
                sendnum = 2058 #was 62
            else:
                sendnum = 2052 #was 58   
            
            if(pillar_frames > 10 and red_pillar_area>400):
                Last_Pillar = "red"
        
        if(final_turnaround >25):
            final_turnaround+=3
            sendnum = str(1400) #was all 1395
            ser.write((sendnum + "\n").encode('utf-8'))
            
            sendnum = 2182
    
    #GREEN PILLAR DETECTION (and maneuvering for green pillars)
    if (green_pillar_area >= 150 and pillar_frames > 9): #movement for detection of green pillar #was 150 | 8, 7 or 6, 9
        #print("green pillar area: ", green_pillar_area)       
        if(red_pillar_area >= 150):
            if(green_y > red_y):
                if(green_pillar_area <= 650):
                    sendnum = 2134 #was 2132
                elif(green_pillar_area <= 1400):
                    sendnum = 2136 #was 2136
                elif(green_pillar_area <= 1900):
                    sendnum = 2140 #was 2134
                elif(green_pillar_area <= 2900):
                    sendnum = 2144
                elif(green_pillar_area <= 3900):
                    sendnum = 2148
                else:
                    sendnum = 2150
                
                if(pillar_frames > 10 and green_pillar_area > 400):
                    Last_Pillar = "green"
                
        else:
            if(green_pillar_area <= 650):
                sendnum = 2134
            elif(green_pillar_area <= 1400):
                sendnum = 2136 
            elif(green_pillar_area <= 1900):
                sendnum = 2140 
            elif(green_pillar_area <= 2900):
                sendnum = 2144
            elif(green_pillar_area <= 3900):
                sendnum = 2148
            else:
                sendnum = 2150
                
            if(pillar_frames > 10 and green_pillar_area > 400):
                Last_Pillar = "green"
         
        if(final_turnaround > 0):
            final_turnaround+=1
            sendnum = str(1395) #was all 1395
            ser.write((sendnum + "\n").encode('utf-8'))
            
            sendnum = 2124
             
        
    #Sends the angle command to the arduino
            
    #print("angle is: ", sendnum)
    sendnum = str(sendnum)
    ser.write((sendnum + "\n").encode('utf-8'))
           
            
    #ENDING SECTION (stops when 3 laps have been completed 
    if (count == 12): #end when see the upcoming line on ground
        FinalFrame+=1
           
        if(FinalFrame > 170):#stops car after it has run 175 frames to make sure it ends at the correct place
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