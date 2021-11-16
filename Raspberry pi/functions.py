# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 00:03:52 2021

@author: Nitro 5
"""
import numpy as np
import math

def mapfun(value,fromLow,fromHigh,toLow,toHigh):
        
    #output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;    
    output =  np.true_divide(np.dot((value - fromLow),(toHigh - toLow)),(fromHigh - fromLow))+ toLow; 
    
    return output


def pol2cart(rho, phi):
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return(x, y)



def cart2sph(x,y,z):
    
    azimuth = np.arctan2(y,x)
    elevation = np.arctan2(z,np.sqrt(x**2 + y**2))
    r = np.sqrt(x**2 + y**2 + z**2)
    return azimuth, elevation, r



def ConvertXYZ(beetle_location,points):
    output =  np.array([points[:,0] - beetle_location[0],points[:,1] -beetle_location[1] ,points[:,2]]).T 
    return output
   
 
def GetPointsForCalibration(PlorPoints,pointsXYZ):
    
    mode = 0
    theta0 = 1/6*math.pi
    phi0 = 0.9*math.pi
    CalPoints_index = np.zeros((6,1))
    CalPoints = np.zeros((18,1))#get the xyz of 6 points
    while mode == 0 :# add the first point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < theta0) and (PlorPoints[i,0] > 0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[0] = pointsXYZ[i,0]
                    CalPoints[1] = pointsXYZ[i,1]
                    CalPoints[2] = pointsXYZ[i,2]
                    CalPoints_index[0] = i
                    mode = 1
                    break
                

                                  
    while mode == 1 :# add the second point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 2*theta0) and (PlorPoints[i,0] > theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[3] = pointsXYZ[i,0]
                    CalPoints[4] = pointsXYZ[i,1]
                    CalPoints[5] = pointsXYZ[i,2]
                    CalPoints_index[1] = i
                    mode = 2
                    break
    
   
               
    while mode == 2 :# add the third point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 3*theta0) and (PlorPoints[i,0] > 2*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[6] = pointsXYZ[i,0]
                    CalPoints[7] = pointsXYZ[i,1]
                    CalPoints[8] = pointsXYZ[i,2]
                    CalPoints_index[2] = i
                    mode = 3
                    break
                
    
                    
    while mode == 3 :# add the fourth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 4*theta0) and (PlorPoints[i,0] > 3*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[9] = pointsXYZ[i,0]
                    CalPoints[10] = pointsXYZ[i,1]
                    CalPoints[11] = pointsXYZ[i,2]
                    CalPoints_index[3] = i
                    mode = 4
                    break
                
    
                    
    while mode == 4 :# add the fifth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 5*theta0) and (PlorPoints[i,0] > 4*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[12] = pointsXYZ[i,0]
                    CalPoints[13] = pointsXYZ[i,1]
                    CalPoints[14] = pointsXYZ[i,2]
                    CalPoints_index[4] = i
                    mode = 5
                    break
                
    
                    
    while mode == 5 :# add the sixth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 6*theta0) and (PlorPoints[i,0] > 5*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0 ):
                    CalPoints[15] = pointsXYZ[i,0]
                    CalPoints[16] = pointsXYZ[i,1]
                    CalPoints[17] = pointsXYZ[i,2]
                    CalPoints_index[5] = i
                    mode = 6
                    break
                
    return CalPoints_index,CalPoints

def GetPointsForVerification(PlorPoints,pointsXYZ):
    
    mode = 0
    theta0 = 1/12*math.pi
    phi0 = 0.95*math.pi
    CalPoints_index = np.zeros((6,1))
    CalPoints = np.zeros((18,1))#get the xyz of 6 points
    while mode == 0 :# add the first point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < theta0) and (PlorPoints[i,0] > 0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] >phi0):
                    CalPoints[0] = pointsXYZ[i,0]
                    CalPoints[1] = pointsXYZ[i,1]
                    CalPoints[2] = pointsXYZ[i,2]
                    CalPoints_index[0] = i 
                    mode = 1
                    break
                

                                  
    while mode == 1 :# add the second point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 2*theta0) and (PlorPoints[i,0] > theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0):
                    CalPoints[3] = pointsXYZ[i,0]
                    CalPoints[4] = pointsXYZ[i,1]
                    CalPoints[5] = pointsXYZ[i,2]
                    CalPoints_index[1] = i
                    mode = 2
                    break
    
   
               
    while mode == 2 :# add the third point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 3*theta0) and (PlorPoints[i,0] > 2*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] >phi0):
                    CalPoints[6] = pointsXYZ[i,0]
                    CalPoints[7] = pointsXYZ[i,1]
                    CalPoints[8] = pointsXYZ[i,2]
                    CalPoints_index[2] = i
                    mode = 3
                    break
                
    
                    
    while mode == 3 :# add the fourth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 4*theta0) and (PlorPoints[i,0] > 3*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] >phi0):
                    CalPoints[9] = pointsXYZ[i,0]
                    CalPoints[10] = pointsXYZ[i,1]
                    CalPoints[11] = pointsXYZ[i,2]
                    CalPoints_index[3] = i
                    mode = 4
                    break
                
    
                    
    while mode == 4 :# add the fifth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 5*theta0) and (PlorPoints[i,0] > 4*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0):
                    CalPoints[12] = pointsXYZ[i,0]
                    CalPoints[13] = pointsXYZ[i,1]
                    CalPoints[14] = pointsXYZ[i,2]
                    CalPoints_index[4] = i
                    mode = 5
                    break
                
    
                    
    while mode == 5 :# add the sixth point
        for i in range(len(PlorPoints)):
            if (PlorPoints[i,0] < 6*theta0) and (PlorPoints[i,0] > 5*theta0):
                if (PlorPoints[i,1] < math.pi) and (PlorPoints[i,1] > phi0):
                    CalPoints[15] = pointsXYZ[i,0]
                    CalPoints[16] = pointsXYZ[i,1]
                    CalPoints[17] = pointsXYZ[i,2]
                    CalPoints_index[5] = i
                    mode = 6
                    break
                
    return CalPoints_index,CalPoints

def GetPanTilt(index,raw):
    index = int(index)
    output1 = int(raw[index*5]*256 + raw[index*5+1])
    output2 = int(raw[index*5+2]*256 + raw[index*5+3])
    output1 = str(output1)
    output2 = str(output2)
    
    for i in range(4-len(output1)):
        output1 = '0'+output1
    for i in range(4-len(output2)):
        output2 = '0'+output2
    return output1,output2
                    
def PanTiltToXYZ(index,raw):
    index = int(index)
    output1 = int(raw[index*5]*256 + raw[index*5+1])
    output2 = int(raw[index*5+2]*256 + raw[index*5+3])
    output1 = str(output1)
    output2 = str(output2)
    
    for i in range(4-len(output1)):
        output1 = '0'+output1
    for i in range(4-len(output2)):
        output2 = '0'+output2
    return output1,output2                
    
def GetAngle(points):
    
    angle_distance = np.zeros((len(points),3))
    for i in range(len(points)):
        [angle_distance[i,0],angle_distance[i,1],angle_distance[i,2]] = \
        cart2sph(points[i,0],points[i,1],points[i,2])
    return angle_distance


def GetLaserTarget(direction,angle_distance,data_from_lidar):
    minimum = 100000  # whatever large number
    for i in range(len(angle_distance)):
         find_minimum = abs(angle_distance[i,0]-direction[0]) + abs(angle_distance[i,1]-direction[1]);
         if (minimum > find_minimum ):
             minimum = find_minimum
             index = i
             output = data_from_lidar[i];   
    return(index,output)
        
    
    
    
    

