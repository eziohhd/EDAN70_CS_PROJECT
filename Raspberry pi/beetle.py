# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from functions import *
import math
import pandas as pd
import numpy as np


df = pd.read_csv(r'C:\Users\Nitro 5\Desktop\EDAN70_CS_PROJECT\EDAN70_CS_PROJECT\matlab\data_test.csv',header=None)   #read the csv file (put 'r' before the path string to address any special characters in the path, such as '\'). Don't forget to put the file name at the end of the path + ".csv"

# df = open('table.csv')
rawM = df.to_numpy()
T = np.zeros((int(rawM.size/5),3))
points_size = int(T.size/3)
points = np.zeros((points_size,3))

for i in range(points_size):
    
    T[i,0] =  mapfun(rawM[5*i,0]*256+rawM[5*i+1,0],0,4096,0,2*math.pi)
    T[i,1] =  mapfun(rawM[5*i+2,0]*256+rawM[5*i+3,0],1024,3072,0,math.pi)
    T[i,2] =  rawM[5*i+4,0]

for i in range(points_size):
    
    points[i,0] = T[i,2]*math.sin(T[i,1])*math.cos(T[i,0])
    points[i,1] = T[i,2]*math.sin(T[i,1])*math.sin(T[i,0])
    points[i,2]=16-math.cos(T[i,1])*T[i,2]

file = GetPointsForCalibration(T,points)
test_index,test = GetPointsForVerification(T,points)
direction = np.array([45/180*math.pi,30/180*math.pi]);  #The angle relationship between the laser and the beetle that we want
beetle_location = [120,300];                  # Detect the beetle location
new_points = ConvertXYZ(beetle_location,points);
angle_distance = GetAngle(new_points) 
[index,laser_target] = GetLaserTarget(direction,angle_distance,points)



