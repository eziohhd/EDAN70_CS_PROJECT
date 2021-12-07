# User Manual

The user manual describes how to use our holodeck system for redirecting dung beetles.

## Hardware setup
The hardware setup has already been done. In case of accidents, web pages related to hardware settings are placed here.
- LIDAR: https://buy.garmin.com/en-US/US/p/578152
- Turret, servos, U2D2, microcontroller, and more: https://www.trossenrobotics.com/p/ScorpionX-RX-64-robot-turret.aspx
- Laser: https://se.rs-online.com/web/p/laser-modules/1271557

## Software setup
### Arduino
For Arduino, the executable file has been written and does not need to be changed. However, if you want to make some modifications to the executable file, you can modify the Dungbeetle.ino file in the DungBeetle folder. If you need to rewrite the ino file into the arduino, you can check this web page to find what you might need : https://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide
### Raspberry pi
In our system, we use python scripts to control the raspberry pi. And the environment and libraries required for the program to run have been configured. In case of raspberry pi is damaged, you can follow the instructions below to install the required libraries.
- OpenCV: https://qengineering.eu/install-opencv-4.5-on-raspberry-pi-4.html
- Inquirer: 
```
pip3 install inquirer
```
- Pandas:
```
sudo apt-get install python3-pandas
```
- Numpy:
```
sudo apt-get install python3-numpy
```
## Instructions about how to run the system
- After connecting the USB port of the arduino to the Raspberry Pi, power on the two microcontrollers.
- Use ssh to control the raspberry pi remotely
- 









