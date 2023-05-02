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

AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);

void setup() {
  // Set the maximum speed in steps per second:
  stepper1.setMaxSpeed(2000);
  stepper2.setMaxSpeed(2000);
  stepper4.setMaxSpeed(2000);
  stepper3.setMaxSpeed(2000);
  
  pinMode(BP1, INPUT);
  pinMode(BP2, INPUT);
  pinMode(BP3, INPUT);
  pinMode(BP4, INPUT);
  pinMode(BP7, INPUT);
  pinMode(BP8, INPUT);
  pinMode(BP5, INPUT);
  pinMode(BP6, INPUT);

  
  Serial.begin(9600);
}
void loop() {
  
  BS1 = digitalRead(BP1);
  BS2 = digitalRead(BP2);
  BS3 = digitalRead(BP3);
  BS4 = digitalRead(BP4);
  BS5 = digitalRead(BP5);
  BS6 = digitalRead(BP6);
  BS7 = digitalRead(BP7);
  BS8 = digitalRead(BP8);
 
  if (BS1 == HIGH) {
    
     stepper1.setSpeed(1500);// Set the speed in steps per second:
     stepper1.runSpeed();// Step the motor with a constant speed as set by setSpeed():

  } else if (BS2 == HIGH){
    
     
     stepper1.setSpeed(-1500);
     stepper1.runSpeed();
     
  }
  if (BS3 == HIGH) {
    
     stepper2.setSpeed(1500);// Set the speed in steps per second:
     stepper2.runSpeed();// Step the motor with a constant speed as set by setSpeed():
     

  } else if (BS4 == HIGH){
     
     stepper2.setSpeed(-1500);
     stepper2.runSpeed();
  } 
   if (BS5 == HIGH) {
     
     stepper3.setSpeed(1500);// Set the speed in steps per second:
     stepper3.runSpeed();// Step the motor with a constant speed as set by setSpeed():
     

     
  } else if (BS6 == HIGH){
    
     
     stepper3.setSpeed(-1500);
     stepper3.runSpeed();
    

  }
   if (BS7 == HIGH) {
    
     stepper4.setSpeed(-1500);// Set the speed in steps per second:
     stepper4.runSpeed();// Step the motor with a constant speed as set by setSpeed():
     
  } else if (BS8 == HIGH){
    

     stepper4.setSpeed(1500);
     stepper4.runSpeed();

  }
}
