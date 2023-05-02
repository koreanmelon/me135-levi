#include <SpeedyStepper.h>
#include <PIDLoop.h>
#include <Pixy2.h>
#include <AccelStepper.h> // Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:

const int BP1 = 44;  
const int BP2 = 45;  
const int BP3 = 42;  
const int BP4 = 43;  
const int BP5 = 40;  
const int BP6 = 41;  
const int BP7 = 38;  
const int BP8 = 39;

int BS1 = 0; 
int BS2 = 0; 
int BS3 = 0; 
int BS4 = 0; 
int BS5 = 0; 
int BS6 = 0; 
int BS7 = 0; 
int BS8 = 0; 

#define dirPin4 32
#define stepPin4 33
#define dirPin1 34
#define stepPin1 35
#define dirPin2 36
#define stepPin2 37
#define dirPin3 30
#define stepPin3 31

#define motorInterfaceType 1// Create a new instance of the AccelStepper class:
//AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
//AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
//AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
//AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);

SpeedyStepper stepper1;
SpeedyStepper stepper2;
SpeedyStepper stepper3;
SpeedyStepper stepper4;



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

  if (token == 0) {

    pixy.ccc.getBlocks();
    xCoordinateTip = pixy.ccc.blocks[0].m_x;
    yCoordinateTip = pixy.ccc.blocks[0].m_y;

    if (pixy.ccc.numBlocks > 1) {
      xCoordinateTracking = pixy.ccc.blocks[0].m_x-41 ;
      yCoordinateTracking = pixy.ccc.blocks[0].m_y+41;
      xCoordinateTip = pixy.ccc.blocks[1].m_x;
      yCoordinateTip = pixy.ccc.blocks[1].m_y;

      Serial.println("found second block");
      check2 = 1;
      if (firstTime == 0) {
        firstTime = 1;
        delay(5000);
      }
    }
    token = 1;  
  }
  
  //should be token == 1
  if (token == 1 && check2 == 1) {
//    Serial.print(xCoordinate);
//    Serial.print(", ");
//    Serial.println(yCoordinate);
//    Serial.println("1");
    // check if in location
    xOffset = abs(xCoordinateTip - xCoordinateTracking);
    yOffset = abs(yCoordinateTip - yCoordinateTracking);
    if ((xOffset < 5)  &&  (yOffset< 5)){
      check1 = 1;
//      delay(500);
      Serial.println("arrived at location allegedly");
      Serial.print("x Offset: ");
      Serial.println(abs((xCoordinateTip - xCoordinateTracking)));
      Serial.print("y Offset: ");
      Serial.println(abs((yCoordinateTip - yCoordinateTracking)));
    } else { // if not, move to the correct location
//      Serial.println("setting up movement");
      if (xCoordinateTip < xCoordinateTracking) {
  //      stepper1.moveRelativeInSteps(-stepSize);
        stepper3.setupRelativeMoveInSteps(-stepSize);
      } else {
  //      stepper1.moveRelativeInSteps(stepSize);
        stepper3.setupRelativeMoveInSteps(stepSize);
      } 
      if (yCoordinateTip < yCoordinateTracking) {
  //      stepper4.moveRelativeInSteps(stepSize);
        stepper4.setupRelativeMoveInSteps(stepSize);
      } else {
  //      stepper4.moveRelativeInSteps(-stepSize);
        stepper4.setupRelativeMoveInSteps(-stepSize);
      }
      while((!stepper1.motionComplete()) || (!stepper4.motionComplete())) {
        stepper3.processMovement();
        stepper4.processMovement();
      }
    }
    
    
    
  }
  token = 0;
    // extend the stage 1
  if (check1 == 1) {
//      stepper2.setupRelativeMoveInSteps(stepSize);
    stepper1.setupRelativeMoveInSteps(stepSize);
    stepper1.processMovement();
    Serial.println("arrived at destination!");
//    delay(500);
    check1 = 0;
  }

    Serial.print("Tracking: ");
    Serial.print(xCoordinateTracking);
    Serial.print(", ");
    Serial.println(yCoordinateTracking);
    Serial.print("TIP: ");
    Serial.print(xCoordinateTip);
    Serial.print(", ");
    Serial.println(yCoordinateTip);
  
  BS1 = digitalRead(BP1);
  BS2 = digitalRead(BP2);
  BS3 = digitalRead(BP3);
  BS4 = digitalRead(BP4);
  BS5 = digitalRead(BP5);
  BS6 = digitalRead(BP6);
  BS7 = digitalRead(BP7);
  BS8 = digitalRead(BP8);
  
  if (BS1 == HIGH) {
     stepper1.setupRelativeMoveInSteps(4*ButtonStepSize);// Set the speed in steps per second:
  } else if (BS2 == HIGH){
     stepper1.setupRelativeMoveInSteps(-4*ButtonStepSize);// Set the speed in steps per second:
  }
  if (BS3 == HIGH) {
     stepper2.setupRelativeMoveInSteps(ButtonStepSize);// Set the speed in steps per second:
  } else if (BS4 == HIGH){
     stepper2.setupRelativeMoveInSteps(-ButtonStepSize);// Set the speed in steps per second:
  } 
   if (BS5 == HIGH) {
     stepper3.setupRelativeMoveInSteps(ButtonStepSize);// Set the speed in steps per second:
  } else if (BS6 == HIGH){
     stepper3.setupRelativeMoveInSteps(-ButtonStepSize);// Set the speed in steps per second:
  }
   if (BS7 == HIGH) {
     stepper4.setupRelativeMoveInSteps(-2*ButtonStepSize);// Set the speed in steps per second:
  } else if (BS8 == HIGH){
     stepper4.setupRelativeMoveInSteps(2*ButtonStepSize);// Set the speed in steps per second:
  }
  while((!stepper1.motionComplete()) || (!stepper2.motionComplete()) || (!stepper3.motionComplete()) || (!stepper4.motionComplete()))
  {
    stepper1.processMovement();
    stepper2.processMovement();
    stepper3.processMovement();
    stepper4.processMovement();
  }
}
