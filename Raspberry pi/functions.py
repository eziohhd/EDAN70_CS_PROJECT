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
        
    
    
    
    

