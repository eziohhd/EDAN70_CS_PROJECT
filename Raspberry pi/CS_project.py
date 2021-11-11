import serial
import time
import cv2
import numpy as np
from numpy.linalg import svd
from numpy.linalg import matrix_rank as rank
from scipy.linalg import diagsvd

# Function to swich between rectangular and polar coordinates need!!


def MotorControl(location1,location2):        
    string_temp = 's'+str(location1)+str(location2)+'\n'
    string = bytes(string_temp,'utf-8')
    ser.write(bytes(string))
    time.sleep(1)       
    return None

# Solve a Homogeneous Linear Equation System: Ax=0
def sol_svd(A):
    A = np.array(A)
    # find the eigenvalues and eigenvector of U(transpose).U
    e_vals, e_vecs = np.linalg.eig(np.dot(A.T, A))  
    # extract the eigenvector (column) associated with the minimum eigenvalue
    return e_vecs[:, np.argmin(e_vals)]

# Solve a SVD System: Ax=b
def sol_svd2(A,b):
    A = np.array(A)
    u,s,vt = svd(A)
    r = rank(A)
    s[:r] = 1/s[:r]
    m,n = A.shape
    s_inv = diagsvd(s,m,n).T
    x = vt.T.dot(s_inv).dot(u.T).dot(b)
    return x

# Matrix A
def Matrix(x,y,z,u,v,A):
    coe1 = [x, y, z, 1, 0, 0, 0, 0, -u*x, -u*y, -u*z, -u]
    coe2 = [0, 0, 0, 0, x, y, z, 1, -v*x, -v*y, -v*z, -v]
    A.append(coe1)
    A.append(coe2)
    return A

# get corresponding cordinate after getting M
def get_corr_point(M,u,v):
    M = M.reshape((3,4))
    p2 = np.array([[u],[v],[1]])
    p3 = sol_svd2(M, p2)
    p3 = p3/p3[-1]
    return p3[0:3]

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
    ser.flush()
    cap = cv2.VideoCapture(0)
    # initialize
    A = []
    cnt = 0
    mode = 1
    led = 14
    tll = 15
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14,GPIO.OUT)
    GPIO.setup(15,GPIO.OUT)
    GPIO.output(led, GPIO.HIGH) 
    #Hsv values for green color 
    lower_green = np.array([40,40,40])
    upper_green = np.array([70,255,255])
    
    #mode 1: hardware setup
    while mode == 1 :
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "Ready":
                ServoReady = 1 
                mode = 2
                
    #mode 2: calibration 
    while mode == 2 :
        # Turn on laser
        GPIO.output(tll, GPIO.HIGH)
        # Capture frame
        _, frame = cap.read()
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv_image, lower_green, upper_green)
        # remove noise
        kernel =  np.ones((5,5),np.uint8)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        cnts,_ = cv2.findContours(green,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
    #     for c in cnts:
        if len(cnts) != 0:
            # find the biggest countour (c) by the area
            c = max(cnts, key = cv2.contourArea)   
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
#             area=cv2.contourArea(c)
        cv2.imshow("Frame", frame)
        cv2.imshow("green", green)
        #if there are data in serial port    
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        # The servos are in position    
            if line == "In position":
                # Do matching when servos are in position
                if len(cnts) != 0:
                    # compute the center of the contour
                    u = round((x + w) / 2)
                    v = round((y + h) / 2)
                    # (x, y, z) in 3D model
                    # (u, v) in frame
                    A = Matrix(x,y,z,u,v,A)
                    cnt = cnt + 1
                    ServoReady = 1
#               else :
#                   raise ValueError("No object detected!")
        # Stop sending command to servos
        # when ServoReady = 0                 
        if ServoReady == 1:
            if cnt == 0 :
                MotorControl(2000,2000)      
                ServoReady = 0
            elif cnt == 1 :
                MotorControl(2500,2500)
                ServoReady = 0
            elif cnt == 2 :
                MotorControl(2700,2700)
                ServoReady = 0
            elif cnt == 3 :
                MotorControl(2800,2800)
                ServoReady = 0
            elif cnt == 4 :
                MotorControl(2800,2800)
                ServoReady = 0
            elif cnt == 5 :
                MotorControl(2800,2800)
                ServoReady = 0    
            elif cnt == 6 :
                MotorControl(2800,2800)
                ServoReady = 0
            elif cnt == 7 :
                # get the transformation matrix M
                #by solving a Homogeneous Linear
                #Equation System
                M = sol_svd(A)
                servoReady = 0              
                
    #mode 3: Simulation
    # When should we enter mode 3?
    while mode == 3 :
        _, frame = cap.read()
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray_image, 80, 255, cv2.THRESH_BINARY)
        threshold=~threshold
        # remove noise
        kernel =  np.ones((5,5),np.uint8)
        threshold = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)
        cnts,_ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(cnts) != 0:
#             for c in cnts:
#                (x, y, w, h) = cv2.boundingRect(c)
#                area=cv2.contourArea(c)   
#                if 50<area<200:
#                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
#                    cx = round((x + w) / 2)
#                    cy = round((y + h) / 2)
#                    print(cx,cy)
            # find the biggest countour (c) by the area
            c = max(cnts, key = cv2.contourArea)   
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
            # compute the center of the contour
            u = round((x + w) / 2)
            v = round((y + h) / 2)
            p3 = get_corr_point(M, 130, 130)
            # Algorithm to redirect dung beetle!!
            
            
            # then
            #MotorControl(pan,tilt)            
                       
            
                      
        cv2.imshow("Frame", frame)
        cv2.imshow("threshold", threshold)
            
            
            
