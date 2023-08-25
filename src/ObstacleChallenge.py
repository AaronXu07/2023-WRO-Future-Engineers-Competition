import cv2
import time
from picamera2 import Picamera2
import serial
import readchar
from readchar import readkey, key
import numpy as np
import RPi.GPIO as GPIO
from time import sleep

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 60
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

lower_black = np.array([0,0,0])
upper_black = np.array([140,255,40])  #home threshold

#upper_black = np.array([170,255,50]) #away
#upper_black = np.array([110,255,30])

derivative = -1
sendnum = 1500
angle = 2098

#color in BGR
YELLOW = (0, 255, 255)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (245, 206, 135)
PINK = (121, 28, 227)
NAVY = (68, 42, 32)
PURPLE = (128, 0, 128)
# Line thickness of 4 px
thickness = 3

prev_frame_time = 0
new_frame_time = 0

count = 0
TurnFrameCount = 0
FinalFrame = 0

Clockwise = False
CounterClockwise = False

Blue_Seen = False
Orange_Seen = False
        
green_pillar_area = 0
red_pillar_area = 0
if __name__ == '__main__':
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    #ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    
    sleep(8)#delay for arduino
    print("ready")

while True:
    # read image from disk
    im= picam2.capture_array()
   
    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps = str(fps)
    cv2.putText(im, fps, (8, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
    
    Left_points = [(10,225), (185,225), (185,320), (10,320)] #left region of interest
    Right_points = [(630,230), (450,230), (450,325), (630,325)] #right region of interest
    
    Mid_points = [(140,210), (515, 210), (515, 405), (140, 405)]
    Turn_points = [(200, 360), (450, 360), (450, 390), (200, 390)]
    
    # Using cv2.line() method to draw a yellow line with thickness of 4 px
    im = cv2.line(im, Left_points[0], Left_points[1], YELLOW, thickness)
    im = cv2.line(im, Left_points[1], Left_points[2], YELLOW, thickness)
    im = cv2.line(im, Left_points[2], Left_points[3], YELLOW, thickness)
    im = cv2.line(im, Left_points[3], Left_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Right_points[0], Right_points[1], YELLOW, thickness)
    im = cv2.line(im, Right_points[1], Right_points[2], YELLOW, thickness)
    im = cv2.line(im, Right_points[2], Right_points[3], YELLOW, thickness)
    im = cv2.line(im, Right_points[3], Right_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Mid_points[0], Mid_points[1], PURPLE, thickness)
    im = cv2.line(im, Mid_points[1], Mid_points[2], PURPLE, thickness)
    im = cv2.line(im, Mid_points[2], Mid_points[3], PURPLE, thickness)
    im = cv2.line(im, Mid_points[3], Mid_points[0], PURPLE, thickness)
    
    im = cv2.line(im, Turn_points[0], Turn_points[1], YELLOW, thickness)
    im = cv2.line(im, Turn_points[1], Turn_points[2], YELLOW, thickness)
    im = cv2.line(im, Turn_points[2], Turn_points[3], YELLOW, thickness)
    im = cv2.line(im, Turn_points[3], Turn_points[0], YELLOW, thickness)
    
    L_subimage = im[225:320, 10:185] #was x 15-245
    R_subimage = im[230:325, 450:630] #was y 225-320 x 380:620
    
    M_subimage = im[205:405, 140:515]
    turn_subimage = im[360:390, 180:480]
    
    
    L_imgHSV = cv2.cvtColor(L_subimage, cv2.COLOR_BGR2HSV)
    R_imgHSV = cv2.cvtColor(R_subimage, cv2.COLOR_BGR2HSV)
    L_mask = cv2.inRange(L_imgHSV, lower_black, upper_black)
    R_mask = cv2.inRange(R_imgHSV, lower_black, upper_black)
    
    
    #left countours
    L_contours = cv2.findContours(L_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    LeftMaxA = 0
    LeftMaxI = 0
    if (len(L_contours) > 0):
        for i in range(len(L_contours)):
            L_cnt = L_contours[i]
            area = cv2.contourArea(L_cnt)
            if(area > LeftMaxA):
                LeftMaxA = area
                LeftMaxI = i     
        
        if (LeftMaxA > 20):
            
            L_cnt = L_contours[LeftMaxI]
            area = cv2.contourArea(L_cnt) 
            #cv2.drawContours(L_subimage, L_contours, LeftMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(L_cnt, 0.01*cv2.arcLength(L_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(L_subimage,(x,y),(x+w,y+h),(255,255,255),2)
            
            left_lane_a = LeftMaxA
            #print("Left Lane Area = " + str(left_lane_a))
            #print("Number of Left Contours found = " + str(len(L_contours)))  
    
    #right countours
    R_contours = cv2.findContours(R_mask, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    #Right countour
    RightMaxA = 0
    RightMaxI = 0 
    if (len(R_contours) > 0):
        for i in range(len(R_contours)):
            R_cnt = R_contours[i]
            area = cv2.contourArea(R_cnt)
            if(area > RightMaxA):
                RightMaxA = area
                RightMaxI = i 
        
        if (RightMaxA > 20):
            
            R_cnt = R_contours[RightMaxI]
            area = cv2.contourArea(R_cnt)
            #cv2.drawContours(R_subimage, R_contours, RightMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(R_cnt, 0.01*cv2.arcLength(R_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(R_subimage,(x,y),(x+w,y+h),(255,255,255),2)
            
            right_lane_a = RightMaxA
            #print("Right Lane Area = " + str(right_lane_a))        
            #print("Number of Right Contours found = " + str(len(R_contours)))
    
    if (len(R_contours) == 0 or RightMaxA <= 70):
        right_lane_a = 0  
    if (len(L_contours) == 0 or LeftMaxA <= 70):
        left_lane_a = 0
        
    
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)
    #lower_blue = np.array([100, 150, 0], np.uint8) #away
    lower_blue = np.array([100, 120, 0], np.uint8) #home
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
            B_cnt = blue_contours[BMaxI]
            area = cv2.contourArea(B_cnt)
            #cv2.drawContours(turn_subimage, blue_contours, BMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(B_cnt, 0.01*cv2.arcLength(B_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)   
            blue_line_a = BMaxA

            #print("Blue Line Area = " + str(blue_line_a))        
            #print("Number of red contours found = " + str(len(red_contours)))
                
    if (len(blue_contours) == 0 or BMaxA <= 70):
        blue_line_a = 0
     
    turn_imgHSV = cv2.cvtColor(turn_subimage, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([8, 75, 50], np.uint8) # was [5, 50, 50]
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
            O_cnt = orange_contours[OMaxI]
            area = cv2.contourArea(O_cnt)
            #cv2.drawContours(turn_subimage, orange_contours, OMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(O_cnt, 0.01*cv2.arcLength(O_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(turn_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)   
            orange_line_a = OMaxA

            #print("Orange Line Area = " + str(orange_line_a))        
            #print("Number of orange contours found = " + str(len(orange_contours)))
                
    if (len(orange_contours) == 0 or OMaxA <= 70):
        orange_line_a = 0
        
        
    M_imgHSV = cv2.cvtColor(M_subimage, cv2.COLOR_BGR2HSV) 
    
    #lower mask (0-10)
    lower_red = np.array([0, 100, 22])
    upper_red = np.array([3, 255, 255])
    mask0 = cv2.inRange(M_imgHSV, lower_red, upper_red)
    
    #upper mask (170-180)
    lower_red = np.array([165, 100, 20])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(M_imgHSV, lower_red, upper_red)      
    raw_mask = mask0 | mask1
    
    red_contours = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find red contours
    
    
    #lower_green = np.array([50,115,27])
    #upper_green = np.array([100,255,255])
    lower_green = np.array([50,125,17])
    upper_green = np.array([90,255,255])
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
        
        if (RedMaxA > 75):
            Red_cnt = red_contours[RedMaxI]
            area = cv2.contourArea(Red_cnt)
            #cv2.drawContours(M_subimage, red_contours, RedMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(Red_cnt, 0.01*cv2.arcLength(Red_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(M_subimage,(x,y),(x+w,y+h),(121, 28, 227),2)   
            red_pillar_area = RedMaxA
            red_y = y
            red_x = x
            #print("Red Pillar Area = " + str(red_pillar_area))        
            #print("Number of red contours found = " + str(len(red_contours)))
    
    GreenMaxA = 0
    GreenMaxI = 0
    if (len(green_contours) > 0):
        for i in range(len(green_contours)):
            G_cnt = green_contours[i]
            area = cv2.contourArea(G_cnt)
            if(area > GreenMaxA):
                GreenMaxA = area
                GreenMaxI = i 
        
        if (GreenMaxA > 75):
            G_cnt = green_contours[GreenMaxI]
            area = cv2.contourArea(G_cnt)
            #cv2.drawContours(M_subimage, green_contours, GreenMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(G_cnt, 0.01*cv2.arcLength(G_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(M_subimage,(x,y),(x+w,y+h),(235, 206, 135),2)   
            green_pillar_area = GreenMaxA
            green_y = y
            green_x = x
            #print("Green Pillar Area = " + str(green_pillar_area))        
            #print("Number of green contours found = " + str(len(red_contours)))
            
    if (len(red_contours) == 0 or RedMaxA <= 80):
        red_pillar_area = 0
    if (len(green_contours) == 0 or GreenMaxA <= 80):
        green_pillar_area = 0
    
    cv2.imshow("Camera", im)
    
    
    sendnum=1385 #starts moving the car forwards (home 1375) (mat 1385)
    sendnum = str(sendnum)
    ser.write((sendnum + "\n").encode('utf-8'))
    
    
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
            
            
    if (right_lane_a >= 0 and left_lane_a >= 0): #wall following only
            
        error = left_lane_a - right_lane_a 
        
        if (derivative == -1):
            derivative = 0
        else:
            derivative = error-prev_error
        
        kp = 0.0075
        kd = 0.0035
        
        steering = int((kp * error) + (kd * derivative))
        
        if (steering > 28): #make sure steering is limited so it doesn't steer too much 
            steering = 28
        elif (steering < -28):
            steering = -28
        
        if (right_lane_a <= 80): #make sure steering is limited so it doesn't steer too much 
            steering = 32
        elif (left_lane_a <= 80):
            steering = -34
        
        sendnum = angle-steering # Greater Than 2100 = to left | Less than 2100 = to the right 
        
        prev_error = error
        
    
    if (red_pillar_area >= 150): #movement for detection of red pillar
        #print("red pillar area: ", red_pillar_area)
        if(green_pillar_area >= 150):
            if(red_y > green_y):
                if(red_pillar_area <= 750):
                    sendnum = 2078
                elif(red_pillar_area <=1500):
                    sendnum = 2076
                elif(red_pillar_area <=2000):
                    sendnum = 2074
                elif(red_pillar_area <=3000):
                    sendnum = 2070
                elif(red_pillar_area <=4000):
                    sendnum = 2066
                else:
                    sendnum = 2060      
        
        else:
            if(red_pillar_area <= 750):
                sendnum = 2078
            elif(red_pillar_area <=1500):
                sendnum = 2076
            elif(red_pillar_area <=2000):
                sendnum = 2074
            elif(red_pillar_area <=3000):
                sendnum = 2070
            elif(red_pillar_area <=4000):
                sendnum = 2066
            else:
                sendnum = 2060     
    
    
    if (green_pillar_area >= 150): #movement for detection of green pillar
        #print("green pillar area: ", green_pillar_area)       
        if(red_pillar_area >= 150):
            if(green_y > red_y):
                if(green_pillar_area <= 750):
                    sendnum = 2118
                elif(green_pillar_area <= 1500):
                    sendnum = 2120
                elif(green_pillar_area <= 2000):
                    sendnum = 2124
                elif(green_pillar_area <= 3000):
                    sendnum = 2128
                elif(green_pillar_area <= 4000):
                    sendnum = 2132
                else:
                    sendnum = 2138
                
        else:
            if(green_pillar_area <= 750):
                sendnum = 2118
            elif(green_pillar_area <= 1500):
                sendnum = 2120
            elif(green_pillar_area <= 2000):
                sendnum = 2124
            elif(green_pillar_area <= 3000):
                sendnum = 2128
            elif(green_pillar_area <= 4000):
                sendnum = 2132
            else:
                sendnum = 2138
                
                
    if (CounterClockwise == True and Blue_Seen): #if needs to turn sharp left
        sendnum = 2136
               
    if (Clockwise == True and Orange_Seen): #if needs to turn sharp right  
        sendnum= 2058
        
    #print("angle is: ", sendnum)
    sendnum = str(sendnum)
    ser.write((sendnum + "\n").encode('utf-8'))
           
            
    #ENDING SECTION
    if (count == 12): #end when see the upcoming line on ground
        FinalFrame+=1
        
        if(CounterClockwise == True):
            
            #final_subimage = im[195:230, 210:390]
            final_subimage = im[220:265, 190:350]
            final_imgHSV = cv2.cvtColor(final_subimage, cv2.COLOR_BGR2HSV)
            
            
            lower_blue = np.array([90, 70, 55], np.uint8) # was 100 (away)
            upper_blue = np.array([140, 255, 255], np.uint8)
            
            maskf = cv2.inRange(final_imgHSV, lower_blue, upper_blue)
            final_contours = cv2.findContours(maskf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours
            
            FMaxA = 0
            FMaxI = 0
            if (len(final_contours) > 0):
                for i in range(len(final_contours)):
                    F_cnt = final_contours[i]
                    area = cv2.contourArea(F_cnt)
                    if(area > FMaxA):
                        FMaxA = area
                        FMaxI = i 
                
                if (FMaxA > 80):
                    F_cnt = final_contours[FMaxI]
                    area = cv2.contourArea(F_cnt)
                    #cv2.drawContours(final_subimage, final_contours, FMaxI, (0, 255, 0), 2)
                    approx=cv2.approxPolyDP(F_cnt, 0.01*cv2.arcLength(F_cnt,True),True)
                    x,y,w,h=cv2.boundingRect(approx)
                    cv2.rectangle(final_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)   
                    final_line_a = FMaxA

                    #print("Final Line Area = " + str(final_line_a))        
                    #print("Number of final contours found = " + str(len(final_contours)))
                        
            if (len(final_contours) == 0 or FMaxA <= 85):
                final_line_a = 0
            
            if(final_line_a > 85):
                sendnum = str(1500)#stops car
                ser.write((sendnum + "\n").encode('utf-8'))
                sendnum = str(2098)
                ser.write((sendnum + "\n").encode('utf-8'))
                break #breaks out of program
            
            elif(FinalFrame > 320):
                sendnum = str(1500)#stops car
                ser.write((sendnum + "\n").encode('utf-8'))
                sendnum = str(2098)
                ser.write((sendnum + "\n").encode('utf-8'))
                break #breaks out of program
            
        if(Clockwise == True):
            #print("ending... orange")
            
            final_subimage = im[210:250, 230:400]
            final_imgHSV = cv2.cvtColor(final_subimage, cv2.COLOR_BGR2HSV)
            
            """
            lower_orange = np.array([5, 50, 50], np.uint8) #home
            upper_orange = np.array([20, 255, 255], np.uint8)
            """
            lower_orange = np.array([8, 15, 165]) #away #was 0 not 8
            upper_orange = np.array([40, 255, 255])
            
            maskf = cv2.inRange(final_imgHSV, lower_orange, upper_orange)
            final_contours = cv2.findContours(maskf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find blue contours
            
            FMaxA = 0
            FMaxI = 0
            if (len(final_contours) > 0):
                for i in range(len(final_contours)):
                    F_cnt = final_contours[i]
                    area = cv2.contourArea(F_cnt)
                    if(area > FMaxA):
                        FMaxA = area
                        FMaxI = i 
                
                if (FMaxA > 80):
                    F_cnt = final_contours[FMaxI]
                    area = cv2.contourArea(F_cnt)
                    #cv2.drawContours(final_subimage, final_contours, FMaxI, (0, 255, 0), 2)
                    approx=cv2.approxPolyDP(F_cnt, 0.01*cv2.arcLength(F_cnt,True),True)
                    x,y,w,h=cv2.boundingRect(approx)
                    cv2.rectangle(final_subimage,(x,y),(x+w,y+h),(255, 255, 255),2)   
                    final_line_a = FMaxA

                    #print("Final Line Area = " + str(final_line_a))        
                    #print("Number of final contours found = " + str(len(final_contours)))
                        
            if (len(final_contours) == 0 or FMaxA <= 85):
                final_line_a = 0
            
            if(final_line_a >= 85):
                sendnum = str(1500)#stops car
                ser.write((sendnum + "\n").encode('utf-8'))
                sendnum = str(2098)
                ser.write((sendnum + "\n").encode('utf-8')) 
                break #breaks out of program
            
            elif(FinalFrame > 340):
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

