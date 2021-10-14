# EDAN70_CS_PROJECT
Author: Haidi Hu, Xingda Li, Sijia Cheng, Xi Chen


## Daily report 0921

  
  - Finished the code for scanning the room. Got motor angles and distance in serial monitor.

  ### Further plan:

  - Export data and generate a 3D plot.



## Daily report 0924

  
  - Create the corresponding github project and update files and libraries(pulsedlight3d and rambo I2C)


  - Use long wires to replace original wires


  - Arduino Serial Output to CSV/Excel(Coolterm)

  ### Further plan:

  - Export data and generate a 3D plot

  - Use raspberry pi + camera to realize the detection of moving objects
  

## Daily report 0928

  - Got the first version of our 3D plot.

  ### Further plan:

  - Make the plot nicer by some sort of calibration or error correction.


## Daily report 0929

  - Able to detect black object and mark with white square for the image taken by pi camera using Matlab.

  ### Further plan:

  - Run detection on live data(images) and get the position of the object in real time.

  
## Daily report 1004

  - Able to detect moving objects captured by the pi camera.(Computer vision toolbox needs to be installed in matlab)
  
  ### Further plan
  
  - Exchange data between the raspberry pi-camera system and the motor-lidar system to control the laser.
  

## Daily report 1006

  - Able to detect and output the position of the black object in real time.
  
  ### Further plan
  
  - To think about the algorithm for deciding the motor movements by the position of the dung beetle.


## Daily report 1012

  - Got the dimension of space using pcfitcuboid function in Matlab.
  
 
## Daily report 1013

  - The reason why previous point cloud differs from an ideal cuboid is the Lidar measurement is floating when the distance is small. So I scanned my room again and got a clean plot, also the dimensions.
  
  ### Further plan
  
  - Link the coordinates of the laser point with the motor angle.
  
 
  









  




