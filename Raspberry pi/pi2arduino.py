import serial
import time

def MotorControl(location1,location2):
        
    string_temp = 's'+str(location1)+str(location2)+'\n'
    string = bytes(string_temp,'utf-8')
    ser.write(bytes(string))
#     if ser.in_waiting > 0: 
    time.sleep(4)       
    return None
    
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
    ser.flush()
#whether the servos are ready to move
    ServoReady = 0
    cnt = 0
    line = ""
#    if ser.isOpen():
#         print(ser.name + ' is open...')
    while True:
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            
        if line == "Ready":
            ServoReady = 1
                
            # The servos are in position    
        if  line == "In position":
            ServoReady = 1
            cnt = cnt + 1                 
                    
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
            
            
            
  

#             
#             if ser.in_waiting > 0:
#                 line_read = ser.readline().decode('utf-8').rstrip()
#                 #line_read = ser.readline()
#                 print(line_read)