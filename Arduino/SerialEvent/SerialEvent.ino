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
#include <I2C.h>
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

//LIDAR lite definitions
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
int distance;
//*****************************
//variables related to scanning
//*****************************
boolean scanStart  = false;     // decide when the sacn should start
boolean calibrationStart  = false;     // decide when the sacn should start

int mappedTiltCurrentPosition=0;
int mappedPanCurrentPosition=0;
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
const int BUFFER_SIZE = 10;
char buf[BUFFER_SIZE];
int incomingByte = 0; 
int p1=PAN_MAX;
int p2=TILT_MAX;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean inputValid = false;
boolean ServoMoving = false;
Commander command = Commander();  //create commander object to accept serial commands

void setup()
{
  Serial.begin(115200); //start serial port communication
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  delay(500);           //delay to let DYNAMIXEL services start  
  
  ax12SetRegister2(1,AX_GOAL_SPEED_L,DEFAULT_SPEED1);    //set the speed on the pan servo
  delay(33);                                             //delay before next DYNAMIXEL command
  ax12SetRegister2(2,AX_GOAL_SPEED_L,DEFAULT_SPEED2);    //set the speed on the tilt servo
  delay(33);                                             //delay before next DYNAMIXEL command
  
  SetPosition(1,PAN_MIN);     //set the position of servo # 1 to its starting/min position
  delay(33);                   //delay before next DYNAMIXEL command
  
  SetPosition(2,TILT_MIN);  //set the position of servo # 2 to  its defualt position. If the servo is not present, nothing will happen  TILT_MAX
  delay(33);                    //delay before next DYNAMIXEL command
  
  
  I2c.begin();     // Opens & joins the i2c bus as master
  delay(100);      // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  //program is ready, turn on the LED
  pinMode(0, OUTPUT);   //set pin mode 
  digitalWrite(0,HIGH); //turn LED on
  Serial.println("Ready");
}

//void setup_scan()
//{
//  Serial.begin(115200); //start serial port communication 115200
//  delay(500);           //delay to let DYNAMIXEL services start  
//  
//  ax12SetRegister2(1,AX_GOAL_SPEED_L,DEFAULT_SPEED1);    //set the speed on the pan servo
//  delay(33);                                             //delay before next DYNAMIXEL command
//  ax12SetRegister2(2,AX_GOAL_SPEED_L,DEFAULT_SPEED2);    //set the speed on the tilt servo
//  delay(33);                                             //delay before next DYNAMIXEL command
//  
//  SetPosition(1,PAN_MIN);     //set the position of servo # 1 to its starting/min position
//  delay(33);                   //delay before next DYNAMIXEL command
//  
//  SetPosition(2,TILT_MIN); //set the position of servo # 2 to  its defualt position. If the servo is not present, nothing will happen
//  delay(33);                    //delay before next DYNAMIXEL command
//
//  I2c.begin();     // Opens & joins the i2c bus as master
//  delay(100);      // Waits to make sure everything is powered up before sending or receiving data  
//  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
//  
//  //program is ready, turn on the LED
//  pinMode(0, OUTPUT);   //set pin mode 
//  digitalWrite(0,HIGH); //turn LED on
//}


void loop() {
  //*detect start scan signal
  if (inputString == "StartScanning")
  {
//      setup_scan();
      scanStart = true;
    // Serial.println("Start scanning");
      inputString = "";
          
  }
  
  if (inputString == "StartCalibration")
  {
      scanStart = false;
      calibrationStart = true;
     // Serial.println("Start calibration");  
      inputString = "";     
  }
  
  
  
  if (scanStart)
  {
//  scan();
  if(mappedTiltCurrentPosition >= TILT_MAX)
  {
    Serial.println("Scan done");
    scanActive = false;
  }
  else
  {
    scanActive = true;
  }
  //Serial.println(scanActive);
  
  if(scanActive)
  {
    //move the pan server
    if(millis() - previousMillisMoving > intervalMoving) 
    {
      // save the last time the servo was polled for moving status
      previousMillisMoving = millis();     
      servoMoving1 =   ax12GetRegister(1, AX_MOVING, 1);
    }
    //compare the currrent number if milliseconds with the last time the distance/position data was reported
    if(millis() - previousMillisReport > intervalReport) 
    {
      panCurrentPosition =   ax12GetRegister(1, AX_PRESENT_POSITION_L, 2);  //get current position from servo
      tiltCurrentPosition =   ax12GetRegister(2, AX_PRESENT_POSITION_L, 2);  //get current position from servo
     
      //Serial.println(panCurrentPosition);
      //Serial.println(tiltCurrentPosition);
      //check that the reported position is within the range (incase of bad return or no return data (-1) )
      if(panCurrentPosition >= PAN_MIN && panCurrentPosition <= PAN_MAX) //
      {
        distance = llGetDistance();   
        //Serial.println(panCurrentPosition);
       // Serial.println(tiltCurrentPosition);
        //get distance from LIDAR lite
        //distance = llGetDistanceAverage(3);                                 //get distance from LIDAR lite, averaging version
        // save the last distance/position was reported
        previousMillisReport = millis();        
        //map the current position to a 0-4095 / 0-360 scale. This helps us work with AX servos with a limited 
        mappedPanCurrentPosition = map(panCurrentPosition, PAN_MIN, PAN_MAX, MAPPED_PAN_MIN, MAPPED_PAN_MAX);
        mappedTiltCurrentPosition = map(tiltCurrentPosition, TILT_MIN, TILT_MAX, MAPPED_TILT_MIN,MAPPED_TILT_MAX);        
             
        Serial.println(highByte(mappedPanCurrentPosition));//position high byte
        Serial.println(lowByte(mappedPanCurrentPosition));//position low byte
        Serial.println(highByte(mappedTiltCurrentPosition));//position high byte
        Serial.println(lowByte(mappedTiltCurrentPosition));//position low byte
        Serial.println(distance);//position low byte

      }
    }
    
     if(servoMoving1 == 0)
      {
        servoMoving2 =   ax12GetRegister(2, AX_MOVING, 1);
        tiltCurrentPosition =   ax12GetRegister(2, AX_PRESENT_POSITION_L, 2);
        
        if(tiltCurrentPosition >= TILT_MIN && tiltCurrentPosition <= TILT_MAX)
        {
        tiltGoalPositon = tiltCurrentPosition + 20;
        if(panGoalPositon == PAN_MIN)
        {
          panGoalPositon = PAN_MAX;
          
        }
        else
        {
          panGoalPositon = PAN_MIN;
        }
        
        delay(5);
        SetPosition(1,panGoalPositon); //set the position of servo # 1 to '0'
        SetPosition(2,tiltGoalPositon); 
        servoMoving1 = 1;
      }

      }
  }
  }
  
  if(calibrationStart)
  {
   servoMonitor();
  if (stringComplete) {
   int strlength = inputString.length();
   if (strlength != 8) {
    Serial.print(" Input invalid: ");
    Serial.println(inputString);
   }
   else {
    inputString.toCharArray(buf,10);
    p1=(buf[0]-48)*1000+(buf[1]-48)*100+(buf[2]-48)*10+(buf[3]-48);
    p2=(buf[4]-48)*1000+(buf[5]-48)*100+(buf[6]-48)*10+(buf[7]-48);
   if(p2>=TILT_MIN && p2<=TILT_MAX) {
    SetPosition(1,p1);
    SetPosition(2,p2); 
    ServoMoving = true;
    Serial.print("Pan angle is ");
    Serial.print(p1, DEC);
    Serial.print(",tilt angle is ");
    Serial.println(p2, DEC);
   }
   }
    //Serial.println(inputString); 
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
}

//*scan function
//void scan()
//{
// 
//if(tiltCurrentPosition == TILT_MAX)
//  {
//    scanActive = false;
//  }
//  else
//  {
//    scanActive = true;
//  }
//  //Serial.println(scanActive);
//  
//  if(scanActive)
//  {
//    //move the pan server
//    if(millis() - previousMillisMoving > intervalMoving) 
//    {
//      // save the last time the servo was polled for moving status
//      previousMillisMoving = millis();     
//      servoMoving1 =   ax12GetRegister(1, AX_MOVING, 1);
//    }
//    
//    //compare the currrent number if milliseconds with the last time the distance/position data was reported
//    if(millis() - previousMillisReport > intervalReport) 
//    {
//      panCurrentPosition =   ax12GetRegister(1, AX_PRESENT_POSITION_L, 2);  //get current position from servo
//      tiltCurrentPosition =   ax12GetRegister(2, AX_PRESENT_POSITION_L, 2);  //get current position from servo
//     
//      //Serial.println(panCurrentPosition);
//      //Serial.println(tiltCurrentPosition);
//      //check that the reported position is within the range (incase of bad return or no return data (-1) )
//      if(panCurrentPosition >= PAN_MIN && panCurrentPosition <= PAN_MAX) //
//      {
//        distance = llGetDistance();   
//        //Serial.println(panCurrentPosition);
//       // Serial.println(tiltCurrentPosition);
//        //get distance from LIDAR lite
//        //distance = llGetDistanceAverage(3);                                 //get distance from LIDAR lite, averaging version
//    
//        // save the last distance/position was reported
//        previousMillisReport = millis();   
//       
//        //map the current position to a 0-4095 / 0-360 scale. This helps us work with AX servos with a limited 
//        int mappedPanCurrentPosition = map(panCurrentPosition, PAN_MIN, PAN_MAX, MAPPED_PAN_MIN, MAPPED_PAN_MAX);
//        int mappedTiltCurrentPosition = map(tiltCurrentPosition, TILT_MIN, TILT_MAX, MAPPED_TILT_MIN,MAPPED_TILT_MAX);        
//       
//        
//        Serial.println(highByte(mappedPanCurrentPosition));//position high byte
//        Serial.println(lowByte(mappedPanCurrentPosition));//position low byte
//        Serial.println(highByte(mappedTiltCurrentPosition));//position high byte
//        Serial.println(lowByte(mappedTiltCurrentPosition));//position low byte
//        Serial.println(distance);//position low byte
//
//      }
//    }
//    
//     if(servoMoving1 == 0)
//      {
//        servoMoving2 =   ax12GetRegister(2, AX_MOVING, 1);
//        tiltCurrentPosition =   ax12GetRegister(2, AX_PRESENT_POSITION_L, 2);
//        
//        if(tiltCurrentPosition >= TILT_MIN && tiltCurrentPosition <= TILT_MAX)
//        {
//        tiltGoalPositon = tiltCurrentPosition + 20;
//        if(panGoalPositon == PAN_MIN)
//        {
//          panGoalPositon = PAN_MAX;
//          
//        }
//        else
//        {
//          panGoalPositon = PAN_MIN;
//        }
//        
//        delay(5);
//        SetPosition(1,panGoalPositon); //set the position of servo # 1 to '0'
//        SetPosition(2,tiltGoalPositon); 
//        servoMoving1 = 1;
//      }
//
//      }
//  }
//}



/*
  monitor the movement of the servos,
  when they stop moving, print "In positon"
*/
void servoMonitor() { 
  if (ServoMoving) {
    servoMoving1 =   ax12GetRegister(1, AX_MOVING, 1);
    servoMoving2 =   ax12GetRegister(2, AX_MOVING, 1);
    if (servoMoving1 == 0 and servoMoving2 == 0) {
      delay(5000);
      Serial.println("In position");
      delay(500);
      ServoMoving = false;
    }
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
    if (not ServoMoving) {
    char inChar = (char)Serial.read(); 
    if (inChar == 's'){
      inputValid= true;
    }
    // add it to the inputString:
    if((inputValid and inChar!='s') and inChar!='\n' and stringComplete == false) {
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
}

// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue){
  
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )

    nackack = I2c.write(LIDARLite_ADDRESS,myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
    
    
  }
}

// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0 ){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
    
  }
  
  return arrayToSave[2]; // Return array for use in other functions
}



int llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function

  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return(distance);
}


int llGetDistanceAverage(int numberOfReadings){ 
  if(numberOfReadings < 2){
    numberOfReadings = 2; // If the number of readings to be taken is less than 2, default to 2 readings
  }
  int sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetDistance(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}
