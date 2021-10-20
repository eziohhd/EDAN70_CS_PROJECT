import serial
import time

def MotorControl(location1,location2):
       
        string_temp = 's'+str(location1)+str(location2)+'\n'
        string = bytes(string_temp,'utf-8')
        nr=ser.write(bytes(string))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(4)
        return None
   
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',9600,timeout=1)
    ser.flush()
   
   
#     if ser.isOpen():
#         print(ser.name + ' is open...')

    while True:
   
        MotorControl(2000,2000)

        MotorControl(2500,2500)

        MotorControl(2700,2700)

        MotorControl(2800,2800)
