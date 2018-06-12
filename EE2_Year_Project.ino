#include <Servo.h>
#include <Wire.h>
#include <Adafruit_CAP1188.h>

// Reset Pin for I2C
#define CAP1188_RESET 4

// For I2C, connect SDA to your Arduino's SDA pin, SCL to SCL pin
// On UNO/Duemilanove/etc, SDA == Analog 4, SCL == Analog 5

// Use I2C, with reset pin
Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_RESET);

// servo
#define numServo 4
#define servoPin0 11
#define servoPin1 10
#define servoPin2 9
#define servoPin3 8

Servo servo0, servo1, servo2, servo3;
byte currentPos[numServo];
int8_t sensor[numServo*2];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CAP1188 test!");

  // Initialize the sensor, if using i2c you can pass in the i2c address
  if (!cap.begin(0x28)) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");

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
  // put your main code here, to run repeatedly:
  uint8_t touch = cap.touched();

  for (uint8_t i = 0; i < numServo << 1; i++){
    sensor[i] =+ (touch & _BV(i)) >> i;
  }
  
  for (uint8_t i = 0; i < numServo; i++){
    if(sensor[i << 1]){
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
