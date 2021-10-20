/*
  Serial Event example
 
 When new serial data arrives, this sketch adds it to a String.
 When a newline is received, the loop prints the string and 
 clears it.
 
 A good test for this is to try it with a GPS receiver 
 that sends out NMEA 0183 sentences. 
 
 Created 9 May 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/SerialEvent
 */
#include <ax12.h>
#include <Commander.h>
#include <stdint.h>

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
const int BUFFER_SIZE = 10;
char buf[BUFFER_SIZE];
int incomingByte = 0; 
int p1=PAN_MAX;
int p2=TILT_MAX;
 
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean inputValid = false;

char MoveDone = "t";// To show that one movement of the motors is done or not

void setup()
{
  Serial.begin(9600); //start serial port communication
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
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
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    int strlength = inputString.length();
   if (strlength != 9) {
    Serial.println("Input invalid");
   }
   else {
    inputString.toCharArray(buf,10);
//    p1=(buf[0]-48)*1000+(buf[1]-48)*100+(buf[2]-48)*10+(buf[3]-48);
//    p2=(buf[4]-48)*1000+(buf[5]-48)*100+(buf[6]-48)*10+(buf[7]-48);
   StringToData(buf,p1,p2);
   if(p2>=TILT_MIN && p2<=TILT_MAX) {
    SetPosition(1,p1);
    SetPosition(2,p2); 
}    
    Serial.print("Pan angle is ");
    Serial.print(p1, DEC);
    Serial.print(",tilt angle is ");
    Serial.println(p2, DEC);
    Serial.println(t);
   }
    //Serial.println(inputString); 
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    if (inChar == 's'){
      inputValid= true;
    }
    // add it to the inputString:
    if(inputValid and inChar!='s') {
     inputString += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      inputValid = false;
    } 
  }
}

/*
 This function translates the string from the raspberry pi to the 
 motor position
 */
 void StringToData(const char *string, int &p1 ,int &p2)
 {
    p1=(string[0]-48)*1000+(string[1]-48)*100+(string[2]-48)*10+(string[3]-48);
    p2=(string[4]-48)*1000+(string[5]-48)*100+(string[6]-48)*10+(string[7]-48); 
 }



