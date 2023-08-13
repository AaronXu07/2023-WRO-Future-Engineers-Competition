import cv2
import time
from picamera2 import Picamera2
import serial
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
upper_black = np.array([170,255,50])

derivative = -1
sendnum = 1500
angle = 2098

#color in BGR
YELLOW = (0, 255, 255)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
# Line thickness of 4 px
thickness = 4

prev_frame_time = 0
new_frame_time = 0

count = 0
TurnFrameCount = 0
FinalFrame = 0

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
    if GPIO.input(5) == GPIO.LOW:
        print("Button pressed")
        break

sleep(2.5)
while True:
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
    
    # Using cv2.line() method to draw a yellow line with thickness of 4 px
    im = cv2.line(im, Left_points[0], Left_points[1], YELLOW, thickness)
    im = cv2.line(im, Left_points[1], Left_points[2], YELLOW, thickness)
    im = cv2.line(im, Left_points[2], Left_points[3], YELLOW, thickness)
    im = cv2.line(im, Left_points[3], Left_points[0], YELLOW, thickness)
    
    im = cv2.line(im, Right_points[0], Right_points[1], YELLOW, thickness)
    im = cv2.line(im, Right_points[1], Right_points[2], YELLOW, thickness)
    im = cv2.line(im, Right_points[2], Right_points[3], YELLOW, thickness)
    im = cv2.line(im, Right_points[3], Right_points[0], YELLOW, thickness)
    """
    
    L_subimage = im[230:325, 20:245] #was x 15-245
    R_subimage = im[230:325, 380:620] #was y 225-320 x 380:620
    
    
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
            """
            cv2.drawContours(L_subimage, L_contours, LeftMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(L_cnt, 0.01*cv2.arcLength(L_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(L_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            """
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
            """
            cv2.drawContours(R_subimage, R_contours, RightMaxI, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(R_cnt, 0.01*cv2.arcLength(R_cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(R_subimage,(x,y),(x+w,y+h),(0,0,255),2)
            """
            right_lane_a = RightMaxA
                    
            #print("Number of Right Contours found = " + str(len(R_contours)))
    
    if (len(R_contours) == 0 or RightMaxA <= 70):
        right_lane_a = 0  
    if (len(L_contours) == 0 or LeftMaxA <= 70):
        left_lane_a = 0
      
      
    #cv2.imshow("L_contours", L_subimage)
    #cv2.imshow("R_contours", R_subimage)
    #cv2.imshow("Camera", im)
    
    #print("Right Lane Area = " + str(right_lane_a))
    #print("Left Lane Area = " + str(left_lane_a))
    
    if(count!=12):
        sendnum=1385#starts moving the car forwards (home 1360) (mat 1375)
        sendnum = str(sendnum)
        ser.write((sendnum + "\n").encode('utf-8'))
    elif(count == 12):
        sendnum=1390#starts moving the car forwards (home 1360) (mat 1375)
        sendnum = str(sendnum)
        ser.write((sendnum + "\n").encode('utf-8'))
        
    if (right_lane_a > 150 and left_lane_a > 150): #if detect both walls
        
        if (right_lane_a > 800 and left_lane_a > 800): #was 850
            if (TurnFrameCount > 11):
                if (count < 12):
                    count+=1
                    print(count)
                    TurnFrameCount = 0
            
        error = left_lane_a - right_lane_a 
        
        if (derivative == -1):
            derivative = 0
        else:
            derivative = error-prev_error
        
        kp = 0.0045
        kd = 0.0040
        
        steering = int((kp * error) + (kd * derivative))
        
        if (steering > 26): #make sure steering is limited so it doesn't steer too much 
            steering = 26
        elif (steering < -26):
            steering = -26
        
        
        sendnum = angle-steering # Greater Than 2100 = to left | Less than 2100 = to the right 
        #print("angle is: ", sendnum)
        sendnum = str(sendnum)
        ser.write((sendnum + "\n").encode('utf-8'))
        
        prev_error = error
        
    elif (right_lane_a > 80 and left_lane_a <= 80): #if detect only right wall
        
        if(right_lane_a < 6000):
            sendnum = 2126

        elif(right_lane_a < 9000):
            sendnum = 2132
        
        elif(right_lane_a >= 9000):
            sendnum = 2138
            
        TurnFrameCount+=1
        #print("angle is: ", sendnum)
        sendnum = str(sendnum)
        ser.write((sendnum + "\n").encode('utf-8'))
            
    elif (right_lane_a <= 80 and left_lane_a > 80): #if detect only left wall
        
        if(left_lane_a < 6000):
            sendnum = 2074

        elif(left_lane_a < 9000):
            sendnum = 2072
        
        elif(left_lane_a >=9000):
            sendnum = 2068
            
        TurnFrameCount+=1
        #print("angle is: ", sendnum)
        sendnum = str(sendnum)
        ser.write((sendnum + "\n").encode('utf-8'))

    
    if (count == 12):
        FinalFrame+=1
        
        if(int(sendnum) > 2090 and int(sendnum) < 2106 and FinalFrame > 12):
            sendnum = str(1500)#stops car
            ser.write((sendnum + "\n").encode('utf-8'))
            sendnum = str(2100)
            ser.write((sendnum + "\n").encode('utf-8')) 
            break #breaks out of program
    
cv2.destroyAllWindows() 