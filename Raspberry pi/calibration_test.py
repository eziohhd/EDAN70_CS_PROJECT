# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 18:53:37 2021

@author: Haidi,Xingda,Sijia,Xi
"""
import datetime
import inquirer
import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import pandas as pd
import random
import sys
from numpy.linalg import svd
from numpy.linalg import matrix_rank as rank
from scipy.linalg import diagsvd
from functions import *

# set servos position
def MotorControl(location1,location2):        
    string_temp = 's'+str(location1)+str(location2)+'\n'
    string = bytes(string_temp,'utf-8')
    ser.write(bytes(string))
    #time.sleep(1)       
    return None

# Solve a Homogeneous Linear Equation System: Ax=0
def sol_svd(A):
    A = np.array(A)
    # find the eigenvalues and eigenvector of U(transpose).U
    e_vals, e_vecs = np.linalg.eig(np.dot(A.T, A))  
    # extract the eigenvector (column) associated with the minimum eigenvalue
    return e_vecs[:, np.argmin(e_vals)]

# Matrix A
def Matrix(x,y,z,u,v,A):
    coe1 = [u, v, 1, 0, 0, 0, 0, 0, 0, -u*x, -v*x, -x]
    coe2 = [0, 0, 0, u, v, 1, 0, 0, 0, -u*y, -v*y, -y]
    coe3 = [0, 0, 0, 0, 0, 0, u, v, 1, -u*z, -v*z, -z]
    A.append(coe1)
    A.append(coe2)
    A.append(coe3)
    return A

# get corresponding cordinate after getting M
def get_corr_point(M,u,v):
    M = M.reshape((4,3))
    x = (M[0][0]*u + M[0][1]*v + M[0][2])/(M[3][0]*u + M[3][1]*v + M[3][2])
    y = (M[1][0]*u + M[1][1]*v + M[1][2])/(M[3][0]*u + M[3][1]*v + M[3][2])
    z = (M[2][0]*u + M[2][1]*v + M[2][2])/(M[3][0]*u + M[3][1]*v + M[3][2])
    p3 = np.array([[x],[y],[z]])
    return p3

# get polar coordinate system and cartesian coordinate system
def raw2polar_cart(rawM):
    
    T = np.zeros((int(rawM.size/5),3))
    points_size = int(T.size/3)
    points = np.zeros((points_size,3))
    for i in range(points_size):      
        T[i,0] =  mapfun(rawM[5*i]*256+rawM[5*i+1],0,4096,0,2*math.pi)
        T[i,1] =  mapfun(rawM[5*i+2]*256+rawM[5*i+3],1024,3072,0,math.pi)
        T[i,2] =  rawM[5*i+4]    
    for i in range(points_size):
        points[i,0] = T[i,2]*math.sin(T[i,1])*math.cos(T[i,0])
        points[i,1] = T[i,2]*math.sin(T[i,1])*math.sin(T[i,0])
        points[i,2]=16-math.cos(T[i,1])*T[i,2]
        
    return T,points


if __name__ == '__main__':
    
    
    cap = cv2.VideoCapture(0)
    
    # initialize
    # change file path
      
    diff = []
    A = []
    points_cloud = []
    # define the number of points we would like to take in calibration
    number_of_samples = 10
    x = 0
    y = 0
    z = 0
    cnt = 0
    mode = 0
    led = 14
    tll = 15
    random_index = 0
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14,GPIO.OUT)
    GPIO.setup(15,GPIO.OUT)
    GPIO.output(led, GPIO.HIGH) 
    #Hsv values for green color 
    lower_green = np.array([40,40,40])
    upper_green = np.array([70,255,255])
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
    ser.flush()
    #mode 0: hardware setup
    while mode == 0 :
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "Ready":
                ServoReady = 1
                questions = [
                inquirer.List('next',
                    message="What do you want to do next?",
                    choices=['Scanning', 'Calibration','Beetle_redirection'],),
                      ]
                answers = inquirer.prompt(questions)
                if answers['next'] == "Scanning":
                    string_scan_temp = 's'+"StartScanning"+'\n'
                    string_scan = bytes(string_scan_temp,'utf-8')
                    ser.write(bytes(string_scan))
                    mode = 1
                else:
                    # Get the data from Lidar and process them to get the polar points and xyz points
                    df = pd.read_csv(r'points_cloud.csv',header=None)   #read the csv file (put 'r' before the path string to address any special characters in the path, such as '\'). Don't forget to put the file name at the end of the path + ".csv"
                    rawM = df.to_numpy()
                    T,points = raw2polar_cart(rawM)
                    if answers['next'] == "Calibration":
                        string_cal_temp = 's'+"StartCalibration"+'\n'
                        string_cal = bytes(string_cal_temp,'utf-8')
                        ser.write(bytes(string_cal))
                        mode = 2
                    else:
                        M_matrix = pd.read_csv(r'Calibration_matrix.csv',header=None)
                        M = M_matrix.to_numpy()
                        string_cal_temp = 's'+"StartCalibration"+'\n'
                        string_cal = bytes(string_cal_temp,'utf-8')
                        ser.write(bytes(string_cal))
                        cnt = 0
                        mode = 4
    
    #mode 1: get points cloud and process them
    while mode == 1 :                   
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "Scan done" :
                np.savetxt('points_cloud.csv', np.array(points_cloud), fmt="%s",delimiter=',')
                sys.exit()
            else :
                points_cloud.append(line)

    
    
    #mode 2: calibration 
    while mode == 2 :
        # Turn off laser
        GPIO.output(tll, GPIO.HIGH)
        # Capture frame
        _, frame = cap.read()
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv_image, lower_green, upper_green)
        # remove noise
        kernel =  np.ones((5,5),np.uint8)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        cnts,_ = cv2.findContours(green,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        if len(cnts) != 0:
            # find the biggest countour (c) by the area
            c = max(cnts, key = cv2.contourArea)   
            (X, Y, W, H) = cv2.boundingRect(c)
            cv2.rectangle(frame, (X, Y), (X + W, Y + H), (255, 0, 0), 3)
#             area=cv2.contourArea(c)
#         cv2.imshow("Frame", frame)
#         cv2.imshow("green", green)
        #if there are data in serial port    
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        # The servos are in position    
            if line == "In position":
                # Do matching when servos are in position
                if len(cnts) != 0:
                    # compute the center of the contour
                    u = round((X + W) / 2)
                    v = round((Y + H) / 2)
                    print(u,v)
                    # (x, y, z) in 3D model
                    # (u, v) in frame
                    # Modify x,y,z
                    x = points[int(random_index),0]
                    y = points[int(random_index),1]
                    z = points[int(random_index),2]
                    print(x,y,z)
                    A = Matrix(x,y,z,u,v,A)
                    cnt = cnt + 1
                    ServoReady = 1
                else :
                  raise ValueError("No object detected!")
        # Stop sending command to servos
        # when ServoReady = 0                 
        if ServoReady == 1:
            if cnt < number_of_samples :
                #get the random points on the roof for calibration
                 random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96)) 
                 pan,tilt = GetPanTilt(random_index,rawM)    
                 MotorControl(pan,tilt)      
                 ServoReady = 0
            else :               
                GPIO.output(tll, GPIO.HIGH)
                M = sol_svd(A)
                print(M)
                np.savetxt('Calibration_matrix.csv', np.array(M), fmt="%s",delimiter=',')
                servoReady = 1
                cnt = 0               
                mode = 3
                
      
    #mode 3: Test
    while mode == 3 :
        # Capture frame
        _, frame = cap.read()
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv_image, lower_green, upper_green)
        # remove noise
        kernel =  np.ones((5,5),np.uint8)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
        cnts,_ = cv2.findContours(green,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        if len(cnts) != 0:
            # find the biggest countour (c) by the area
            c = max(cnts, key = cv2.contourArea)   
            (X, Y, W, H) = cv2.boundingRect(c)
            cv2.rectangle(frame, (X, Y), (X + W, Y + H), (255, 0, 0), 3)
#             area=cv2.contourArea(c)
#         cv2.imshow("Frame", frame)
#         cv2.imshow("green", green)
        #if there are data in serial port    
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        # The servos are in position    
            if line == "In position":
                # Do matching when servos are in position
                if len(cnts) != 0:
                    # compute the center of the contour
                    u = round((X + W) / 2)
                    v = round((Y + H) / 2)
                    p3 = get_corr_point(M, u, v)
                    
                    p3 = np.real(p3)
                    print(u,v)
                    print(p3)
                    # Add x, y, z
                    x = points[int(random_index),0] 
                    y = points[int(random_index),1]
                    z = points[int(random_index),2]
                    print(x,y,z)
                    p3_real = np.array([[x], [y], [z]])
                    diff.append(p3-p3_real)                    
                    # (x, y, z) in 3D model
                    # (u, v) in frame
                    # Modify x,y,z
                   # A = Matrix(x,y,z,u,v,A)
                    cnt = cnt + 1
                    ServoReady = 1
                else :
                  raise ValueError("No object detected!")
        # Stop sending command to servos
        # when ServoReady = 0                 
        if ServoReady == 1:
            if cnt == 0 :
                #get the random points on the roof for calibration verification
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96)) 
                pan,tilt = GetPanTilt(random_index,rawM)  
                MotorControl(pan,tilt)      
                ServoReady = 0
            elif cnt == 1 :
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96))
                pan,tilt = GetPanTilt(random_index,rawM) 
                MotorControl(pan,tilt) 
                ServoReady = 0
            elif cnt == 2 :
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96))
                pan,tilt = GetPanTilt(random_index,rawM) 
                MotorControl(pan,tilt) 
                ServoReady = 0
            elif cnt == 3 :
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96))
                pan,tilt = GetPanTilt(random_index,rawM) 
                MotorControl(pan,tilt) 
                ServoReady = 0
            elif cnt == 4 :
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96))
                pan,tilt = GetPanTilt(random_index,rawM) 
                MotorControl(pan,tilt) 
                ServoReady = 0
            elif cnt == 5 :
                random_index = random.randrange(round(rawM.size/5*0.95),round(rawM.size/5*0.96))
                pan,tilt = GetPanTilt(random_index,rawM) 
                MotorControl(pan,tilt) 
                ServoReady = 0     
            elif cnt == 6 :
                ServoReady = 0
                print(diff)
                # Turn off laser
                GPIO.output(tll, GPIO.LOW)
                sys.exit()
                            
      
# beetle test             
    while mode == 4 :
        
        GPIO.output(tll,GPIO.HIGH)
        direction = np.array([0,90/180*math.pi])
        phi = 0     #The angle between the movement direction of the dung beetle and the positive X axis
        beetle_moving_direction = phi
        direction = direction + phi # Rotate 
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        # The servos are in position    
            if line == "In position":
                cnt = cnt + 1
                ServoReady = 1
                end = datetime.datetime.now()
                period = end - start
                print(int(period.total_seconds()*1000),"ms")
                print("in position")
                    
            if ServoReady == 1:
                if cnt < 20:
                    random_x = random.randrange(0,640)
                    random_y = random.randrange(0,480)
                    print("location on image ",random_x, random_y)
                    random_location = get_corr_point(M, random_x, random_y)
                    beetle_location_x = random_location[0]
                    beetle_location_y = random_location[1]
                    beetle_location = [beetle_location_x,beetle_location_y]; # Detect the beetle location
                    print("beetle_location",beetle_location)
                    new_points = ConvertXYZ(beetle_location,points)
                    angle_distance = GetAngle(new_points) #new_points 
                    [index,laser_target] = GetLaserTarget(direction,angle_distance,points)
                    print("laser target ",laser_target)
                    pan,tilt = GetPanTilt(index,rawM)
                    start = datetime.datetime.now()
                    MotorControl(pan,tilt)
                    ServoReady = 0
    
            
            
            
