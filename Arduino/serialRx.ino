/***************************
 * servo movement
 * This sketch sends positional commands to the AX servo 
 * attached to it - the pan servo must set to ID # 1
 * -the tilt servo set to ID #2
 * The sketch will let the servo 1 rotate full cycle and servo 2 goes up 1 resolution
 * 'For' loops are used to increment and decrement the value of 'i'
 ***************************/
 
 //import ax12 library to send DYNAMIXEL commands
#include <ax12.h>
#include <Commander.h>

#define PAN_MIN  0      //Minimum Servo Position
#define PAN_MAX  4095   //Maximum Servo Position
#define MAPPED_PAN_MIN 0  //Minimum Physical Position on a scale from 0-4096. Mapped value is the same as the PAN value for MX servos
#define MAPPED_PAN_MAX 4095 //Maximum Physical Position on a scale from 0-4096.  Mapped value is the same as the PAN value for MX servos
#define TILT_MIN  2000      //Minimum Tilt Servo Position
#define TILT_MAX  3000  //Maximum Tile Servo Position The 
#define MAPPED_TILT_MIN 2000  //Minimum Physical Position on a scale from 0-2048. Mapped value is the same as the TILT value for MX servos
#define MAPPED_TILT_MAX 3000 //Maximum Physical Position on a scale from 0-4096.  Mapped value is the same as the PAN value for MX servos

#define DEFAULT_SPEED1 30
#define DEFAULT_SPEED2 30



int servoMoving1 = 0;            //whether or not the servo is moving, 0 = not moving, 1 = moving
int panGoalPositon = PAN_MIN;   // Goal Position, where we want the servo to go
int panCurrentPosition;         //Actual position of the servo
int servoMoving2 = 0;            //whether or not the servo is moving, 0 = not moving, 1 = moving
int tiltGoalPositon = TILT_MIN;   // Goal Position, where we want the servo to go
int tiltCurrentPosition;         //Actual position of the servo


long previousMillisMoving = 0;  // last time we requested movement data from the servo
long previousMillisReport = 0;  //  last time distance/angle was reported out to the serial port
int intervalMoving = 50;        // interval at which to request moving data from the servo
int intervalReport = 33;        // interval at which to report the distance/angle data 30hz

boolean scanActive = false;     //false if the servo is not activley scanning, true if it is activley scanning

Commander command = Commander();  //create commander object to accept serial commands
const int BUFFER_SIZE = 8;
int buf[BUFFER_SIZE];
int incomingByte = 0; 
 int  p1=PAN_MAX;
 int p2=TILT_MAX;
void setup()
{
  Serial.begin(115200); //start serial port communication
  delay(500);           //delay to let DYNAMIXEL services start  
  
  ax12SetRegister2(1,AX_GOAL_SPEED_L,DEFAULT_SPEED1);    //set the speed on the pan servo
  delay(33);                                             //delay before next DYNAMIXEL command
  ax12SetRegister2(2,AX_GOAL_SPEED_L,DEFAULT_SPEED2);    //set the speed on the tilt servo
  delay(33);                                             //delay before next DYNAMIXEL command
  
  SetPosition(1,PAN_MAX);     //set the position of servo # 1 to its starting/min position
  delay(33);                   //delay before next DYNAMIXEL command
  
  SetPosition(2,TILT_MAX); //set the position of servo # 2 to  its defualt position. If the servo is not present, nothing will happen
  delay(33);                    //delay before next DYNAMIXEL command


  
  //program is ready, turn on the LED
  pinMode(0, OUTPUT);   //set pin mode 
  digitalWrite(0,HIGH); //turn LED on
}// end setup

void loop()
{
  for(int i=0;i<8;i++)
  {
  if (Serial.available() > 0) {
    // read the incoming byte:
   incomingByte = Serial.read();
     buf[i]=incomingByte-48;
     Serial.print("I received: ");
    Serial.println(buf[i], DEC);
  }
   
  }
 p1=buf[0]*1000+buf[1]*100+buf[2]*10+buf[3];
   p2=buf[4]*1000+buf[5]*100+buf[6]*10+buf[7];
    Serial.println(p1);
      Serial.println(p2);
        delay(500);
      if(p2>=TILT_MIN && p2<=TILT_MAX)
   {SetPosition(1,p1);
     SetPosition(2,p2);}
//  if (Serial.available() >= 5) {
//    // read the incoming bytes:
//    int rlen = Serial.readBytes(buf, BUFFER_SIZE);
//    Serial.print("I received: \n");
//    Serial.print(rlen);
//    for(int i = 0; i < rlen; i++)
//    {
//      Serial.print(buf[i]);
//    }
//    Serial.print("here\n");
//    servoMoving1 =   ax12GetRegister(1, AX_MOVING, 1);
//    servoMoving2 =   ax12GetRegister(2, AX_MOVING, 1);
//    Serial.print("here1\n");
//     Serial.print(servoMoving1);
//     Serial.print(servoMoving2);
//     Serial.print("\n");
//    
//     Serial.print(buf[0]);
//       Serial.print("\n");
//     Serial.print(buf[1]);
//       Serial.print("\n");
//       int p1=buf[0]*256+buf[1];
//        Serial.print("\n");
//       int p2=buf[2]*256+buf[3];
//        Serial.print("\n");
//       SetPosition(1,p1); //set the position of servo # 1 to '0'
//         delay(1000); 
//        SetPosition(2,p2); 
//        delay(1000); 
//    
//  }
  
 
  }
  
  
  
