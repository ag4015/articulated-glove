#include <Wire.h>
#include <Servo.h>
#include "Adafruit_MPR121.h"

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// servo
#define numServo 4
#define servoPin0 11
#define servoPin1 10
#define servoPin2 9
#define servoPin3 8

Servo servo0, servo1, servo2, servo3;
byte currentPos[numServo];
int8_t sensor[numServo*2];

// functions
//void working();
//void debug();

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");

  // Servo
  servo0.attach(servoPin0);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  pinMode(13,OUTPUT);

  for (int i = 0; i < numServo; i++){
    currentPos[i] = 94;
  }
}

void loop() {
  // functioning code
  ;working();
  // debugging mode
  debug();
}

void working(){
  uint8_t touch = cap.touched();

  for (uint8_t i = 0; i < numServo; i++){
    sensor[i] =+ (touch & _BV(i)) >> i;
    if(sensor[i]){
      currentPos[i]++;
    } else {
      currentPos[i] = currentPos[i] - 50;
    }
    currentPos[i] = constrain(currentPos[i], 94, 100);
  }
  
  for (uint8_t i = 0; i < numServo-1; i++){
    Serial.print(currentPos[i]);
    Serial.print("\t");
  }
  Serial.println(currentPos[numServo-1]);
  
  servo0.write(currentPos[0]);
  servo1.write(currentPos[1]);
  servo2.write(currentPos[2]);
  servo3.write(currentPos[3]);
}

void debug(){
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x"); Serial.println(cap.touched(), HEX);
  Serial.print("Filt: ");
  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.filteredData(i)); Serial.print("\t");
  }
  Serial.println();
  Serial.print("Base: ");
  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.baselineData(i)); Serial.print("\t");
  }
  Serial.println();
  
  // put a delay so it isn't overwhelming
  delay(500);
}

