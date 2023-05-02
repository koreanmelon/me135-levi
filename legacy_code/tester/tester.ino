#include <SpeedyStepper.h>
#include <PIDLoop.h>
#include <Pixy2.h>
#include <AccelStepper.h> // Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:

#include <Wire.h>
#include <Servo.h>
#include "mpu9250.h"

const int BP1 = 44;  
const int BP2 = 45;  
const int BP3 = 42;  
const int BP4 = 43;  
const int BP5 = 40;  
const int BP6 = 41;  
const int BP7 = 38;  
const int BP8 = 39;

float xmagOffset = 0;
float ymagOffset = 0;
float zmagOffset = 0;
float xmag = 0;
float ymag = 0;
float zmag = 0;

int BS1 = 0; 
int BS2 = 0; 
int BS3 = 0; 
int BS4 = 0; 
int BS5 = 0; 
int BS6 = 0; 
int BS7 = 0; 
int BS8 = 0; 

int token1 = 0;
int cmd = 0;

int servoPin1 = 9;
int servoPin2 = 10;
Servo Servo1;
Servo Servo2;
float angle1 = 90;
float angle2 = 90;
int servoTolerance = 2;

#define dirPin4 32
#define stepPin4 33
#define dirPin1 34
#define stepPin1 35
#define dirPin2 36
#define stepPin2 37
#define dirPin3 30
#define stepPin3 31

#define openDoorPin 49

#define motorInterfaceType 1// Create a new instance of the AccelStepper class:
//AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
//AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
//AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
//AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);

SpeedyStepper stepper1;
SpeedyStepper stepper2;
SpeedyStepper stepper3;
SpeedyStepper stepper4;

/* Mpu9250 object, I2C bus,  0x68 address */
bfs::Mpu9250 imu(&Wire, 0x68);

// This is the main Pixy object 
Pixy2 pixy;

int xCoordinate = 150;
int yCoordinate = 100;
int token = 0;
int check1 = 0;
int check2 = 0;
int firstTime = 0;
int xOffset = 0;
int yOffset = 0;

int xCoordinateTip = 150;
int xCoordinateTracking = 150;
int yCoordinateTip = 100;
int yCoordinateTracking = 100;

int maxSpeedy = 500000;
float maxAccel = 100000;
int stepSize = 500;
int ButtonStepSize = 500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Starting...\n");

  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  Servo1.write(int(angle1));
  Servo2.write(int(angle2));
  Serial.println("Servos Initialized");

  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
//
  if (imu.Read()) {

    xmagOffset = imu.mag_x_ut();
    ymagOffset = imu.mag_y_ut();
    zmagOffset = imu.mag_z_ut();
   
  }
  
  stepper1.connectToPins(stepPin1, dirPin1);
  stepper2.connectToPins(stepPin2, dirPin2);
  stepper3.connectToPins(stepPin3, dirPin3);
  stepper4.connectToPins(stepPin4, dirPin4);
  
  stepper1.setSpeedInStepsPerSecond(maxSpeedy);
  stepper1.setAccelerationInStepsPerSecondPerSecond(maxAccel);
  stepper2.setSpeedInStepsPerSecond(maxSpeedy);
  stepper2.setAccelerationInStepsPerSecondPerSecond(maxAccel);
  stepper3.setSpeedInStepsPerSecond(maxSpeedy);
  stepper3.setAccelerationInStepsPerSecondPerSecond(maxAccel);
  stepper4.setSpeedInStepsPerSecond(maxSpeedy);
  stepper4.setAccelerationInStepsPerSecondPerSecond(maxAccel);
  
  pixy.init();
  pixy.changeProg("color_connected_components");

  pixy.setServos(500, 600);
  
//  stepper1.setMaxSpeed(2000);
//  stepper2.setMaxSpeed(2000);
//  stepper3.setMaxSpeed(2000);
//  stepper4.setMaxSpeed(2000);
//  
  pinMode(BP1, INPUT);
  pinMode(BP2, INPUT);
  pinMode(BP3, INPUT);
  pinMode(BP4, INPUT);
  pinMode(BP7, INPUT);
  pinMode(BP8, INPUT);
  pinMode(BP5, INPUT);
  pinMode(BP6, INPUT);
  pinMode(openDoorPin, OUTPUT);
//  
//  delay(100);
//
////  stage 1 up
//  servo2.write(90);
//  //stage 1 right
//  servo3.write(90);
//  //stage 2 left
//  servo0.write(90);
//  //stage 2 down
//  servo1.write(60);

}

void loop() {

    digitalWrite(openDoorPin, HIGH);

}
