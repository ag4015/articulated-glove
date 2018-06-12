/*********************************************************
This is a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop 
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required 
to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution
**********************************************************/

#include <Wire.h>
#include <Servo.h>
#include "Adafruit_MPR121.h"
#include <MyoBridge.h>
#include <SoftwareSerial.h>

//MYO BAND INITIALISATION
//SoftwareSerial connection to MyoBridge
//SoftwareSerial bridgeSerial(2,3);
//initialize MyoBridge object with software serial connection
//MyoBridge bridge(bridgeSerial);
//declare a function to handle pose data
//void handlePoseData(MyoPoseData& data) { 
  //convert pose data to MyoPose
  //MyoPose pose;
  //pose = (MyoPose)data.pose;
  //print the pose
  //Serial.println(bridge.poseToString(pose));
//}

//MOTORS INITIALISATION
#define numServo 4
#define servoPin0 12
#define servoPin1 9
#define servoPin2 10
#define servoPin3 6
#define ModeOfOpperation A3
#define furthestPosition 10
#define nearestPosition -2

Servo servo0, servo1, servo2, servo3;
double position(int speed_current, int speed_last, int last_position, int last_time, int current_time, double r);                                        
byte currentSpeed[numServo];
byte lastSpeed[numServo];
int lastPosition[numServo];
int currentPosition[numServo];
unsigned long lasttime = 0;
unsigned long currenttime = 0;
double r = 0.25/1579;
bool d = false;



//SENSORS INITIALISATION
// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();
// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;



//----------------------------------SETUP-------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  //bridgeSerial.begin(115200);
  //wait until MyoBridge has found Myo and is connected. Make sure Myo is not connected to anything else and not in standby!
  //Serial.println("Searching for Myo...");
  //bridge.begin();
  //Serial.println("connected!");
  //set the function that handles pose events
  //bridge.setPoseEventCallBack(handlePoseData);
  //tell the Myo we want Pose data
  //bridge.enablePoseData();
  //make sure Myo is unlocked
  //bridge.unlockMyo();
  
  //You have to perform the sync gesture to receive Pose data!

  
  //SERVO MOTOR SETUP
  servo0.attach(servoPin0);                //Pin Assignment for Servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  pinMode(13,OUTPUT);
  for(int i = 0; i<numServo; i++){
    lastSpeed[i] = 95;   
    currentSpeed[i] = 95;
    lastPosition[i] = 0;
    currentPosition[i] = 0;    
  }

  
  //SENSOR SETUP
  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  } 
  Serial.println("Adafruit MPR121 Capacitive Touch sensor test");  
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
  //delay(10000);
}
 
//--------------------------------------------MAIN LOOP------------------------------------------------
void loop() {
  //bridge.update();

  
  // Get the currently touched pads
  currtouched = cap.touched();
  currenttime = millis();
  
  for (uint8_t i=0; i<numServo; i++) {
    lastSpeed[i] = currentSpeed[i];
    lastPosition[i] = currentPosition[i];
    
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print("Sensor ");Serial.print(i); Serial.println(" touched");
      currentSpeed[i] = currentSpeed[i] + 100;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print("Sensor ");Serial.print(i); Serial.println(" released");
      currentSpeed[i] = currentSpeed[i] - 84;
    }
    if((currtouched & _BV(i))){ //(currtouched & _BV(i))||((currentPose == fist)&&(analogueRead(A3) != 0))
      currentSpeed[i] = currentSpeed[i] + 100;
      Serial.print("Sensor ");Serial.print(i); Serial.println(" held");
    }
    else{
      currentSpeed[i] = currentSpeed[i] - 84;
      Serial.print("Sensor ");Serial.print(i); Serial.println(" not touched");     
    }
    currentSpeed[i] = constrain(currentSpeed[i], 85, 150);
    
    currentPosition[i] = position(currentSpeed[i], lastSpeed[i], lastPosition[i], lasttime, currenttime, r);
    
    //Serial.print("currentSpeed = ");Serial.print(currentSpeed[i]);
    //Serial.print("currentPosition = ");Serial.print(currentPosition[i]);
    //Serial.print("\tfurthestPosition = ");Serial.print(furthestPosition);
    //Serial.print("\tnearestPostion = ");Serial.print(nearestPosition);
    if((currentPosition[i] > furthestPosition)||(currentPosition[i] < nearestPosition)||((currentPosition[i] < nearestPosition)&&(currentSpeed[i] < 95))){
      currentSpeed[i] = 95;
      //Serial.print("Sensor ");Serial.print(i); Serial.print(" left bounds\t");
      //if((currentPosition[i] > furthestPosition)){
        //Serial.println(" - too far");
      //}
      //if((currentPosition[i] < nearestPosition)){
        // Serial.println(" - too near"); 
      //}
    }
      currentPosition[i] = position(currentSpeed[i], lastSpeed[i], lastPosition[i], lasttime, currenttime, r);
  }
  
  
  // reset our state
  lasttouched = currtouched;
    
  
  for (uint8_t i=0; i<numServo; i++) {        //Debugging
    Serial.print("currentSpeed[");Serial.print(i);Serial.print("] = ");Serial.print(currentSpeed[i]);
    //Serial.print("\t lastSpeed[");Serial.print(i);Serial.print("] = ");Serial.print(lastSpeed[i]);
    Serial.print("\t\t currentPosition[");Serial.print(i);Serial.print("] = ");Serial.println(currentPosition[i]);
    //Serial.print("\t lastPosition[");Serial.print(i);Serial.print("] = ");Serial.println(lastPosition[i]);
    //Serial.print("\t\t currenttime = ");Serial.print(currenttime);Serial.print("\t lasttime = ");Serial.print(lasttime);
    //Serial.print("\t r = "); Serial.println(r);
  }
  Serial.println("");
  lasttime = currenttime;
  
  delay(1250);
  servo0.write(currentSpeed[0]);           //Change motor speeds based on the data 
  servo1.write(currentSpeed[1]);           // found from the sensors
  servo2.write(currentSpeed[2]);
  servo3.write(currentSpeed[3]);

  
  return;
}


double position(int speed_current, int speed_last, int last_position, int last_time, int current_time, double r){  //Returns the current position based on the last position,                                        
   double currentRPS = speedToRPS(speed_current);
   double lastRPS = speedToRPS(speed_last);
   double NoofRevolutions = 0.5*(currentRPS + lastRPS)*(current_time-last_time)/1000;    // the last speed and the current speed.
   double current_position = last_position + (2*3.1415*r*NoofRevolutions);
   return current_position;
}

int speedToRPS(int speed_current_last){   //Takes in inputs to the servo motor from currentSpeed
  if(speed_current_last < 75){            // and returns a value in revolutions per second
    return 3.3333;
  }
  else if(speed_current_last == 75){
    return 2.1008;
  }
  else if(speed_current_last == 76){
    return 2.0080;
  }
  else if(speed_current_last == 77){
    return 2.0408;
  }
  else if(speed_current_last == 78){
    return 2.0202;
  }
  else if(speed_current_last == 80){
    return 2.022;
  }
  else if(speed_current_last == 81){
    return 1.999;
  }
  else if(speed_current_last == 82){
    return 1.932;
  }
  else if(speed_current_last == 83){
    return 1.924;
  }
  else if(speed_current_last == 84){
    return 1.873;
  }
  else if(speed_current_last == 85){
    return 1.814;
  }
  else if(speed_current_last == 86){
    return 1.736;
  }
  else if(speed_current_last == 87){
    return 1.665;
  }
  else if(speed_current_last == 88){
    return 1.538;
  }
  else if(speed_current_last == 89){
    return 1.408;
  } 
  else if(speed_current_last == 90){
    return 1.185;
  } 
  else if(speed_current_last == 91){
    return 0.937;
  }
  else if(speed_current_last == 92){
    return 0.672;
  }
  else if(speed_current_last == 93){
    return 0.370;
  }
  else if(speed_current_last == 94){
    return 0.075;
  }
  else if(speed_current_last == 95){
    return 0.00;
  }
  else if(speed_current_last == 96){
    return 0.071;
  }
  else if(speed_current_last == 97){
    return 0.397;
  }
  else if(speed_current_last == 98){
    return 0.676;
  }
  else if(speed_current_last == 99){
    return 122;
  }
  else if(speed_current_last == 100){
    return 122;
  }
  else if(speed_current_last == 101){
    return 122;
  }
  else if(speed_current_last == 102){
    return 122;
  }
  else if(speed_current_last == 103){
    return 122;
  }
  else if(speed_current_last == 104){
    return 122;
  }
  else if(speed_current_last == 105){
    return 122;
  }
  else if(speed_current_last == 106){
    return 122;
  }
  else if(speed_current_last == 107){
    return 122;
  }
  else if(speed_current_last == 108){
    return 122;
  }
  else if(speed_current_last == 109){
    return 122;
  }
  else if(speed_current_last == 110){
    return 122;
  }
  else if(speed_current_last == 111){
    return 122;
  }
  else if(speed_current_last == 112){
    return 122;
  }
  else if(speed_current_last == 113){
    return 122;
  }
  else if(speed_current_last == 114){
    return 122;
  }
  else if(speed_current_last == 115){
    return 122;
  }
  else if(speed_current_last > 115){
    return -200;
  }
}

