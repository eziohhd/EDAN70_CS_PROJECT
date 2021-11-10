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
#include <I2C.h>
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

//LIDAR lite definitions
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
int distance;

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


void setup()
{
  Serial.begin(9600); //start serial port communication 115200
  delay(500);           //delay to let DYNAMIXEL services start  
  
  ax12SetRegister2(1,AX_GOAL_SPEED_L,DEFAULT_SPEED1);    //set the speed on the pan servo
  delay(33);                                             //delay before next DYNAMIXEL command
  ax12SetRegister2(2,AX_GOAL_SPEED_L,DEFAULT_SPEED2);    //set the speed on the tilt servo
  delay(33);                                             //delay before next DYNAMIXEL command
  
  SetPosition(1,PAN_MIN);     //set the position of servo # 1 to its starting/min position
  delay(33);                   //delay before next DYNAMIXEL command
  
  SetPosition(2,TILT_MIN); //set the position of servo # 2 to  its defualt position. If the servo is not present, nothing will happen
  delay(33);                    //delay before next DYNAMIXEL command

    I2c.begin();     // Opens & joins the i2c bus as master
  delay(100);      // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  
  //program is ready, turn on the LED
  pinMode(0, OUTPUT);   //set pin mode 
  digitalWrite(0,HIGH); //turn LED on
}// end setup

void loop()
{
  if(tiltCurrentPosition == TILT_MAX)
  {
    scanActive = false;
  }
  else
  {
    scanActive = true;
  }
  
  if(scanActive == true)
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
      //check that the reported position is within the range (incase of bad return or no return data (-1) )
      if(panCurrentPosition >= PAN_MIN && panCurrentPosition <= PAN_MAX)
      {
        distance = llGetDistance();                                           //get distance from LIDAR lite
        //distance = llGetDistanceAverage(3);                                 //get distance from LIDAR lite, averaging version
    
        // save the last distance/position was reported
        previousMillisReport = millis();   
       
        //map the current position to a 0-4095 / 0-360 scale. This helps us work with AX servos with a limited 
        int mappedPanCurrentPosition = map(panCurrentPosition, PAN_MIN, PAN_MAX, MAPPED_PAN_MIN, MAPPED_PAN_MAX);
        int mappedTiltCurrentPosition = map(tiltCurrentPosition, TILT_MIN, TILT_MAX, MAPPED_TILT_MIN,MAPPED_TILT_MAX);        
       
        
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
  
  
  
  /**
  /* ==========================================================================================================================================
Basic read and write functions for LIDAR-Lite, waits for success message (0 or ACK) before proceeding
=============================================================================================================================================*/

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

  
