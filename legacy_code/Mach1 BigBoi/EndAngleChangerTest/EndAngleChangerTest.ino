// CODE MEANT FOR ARDUINO


//#define step0 13
//#define step1 12
//#define step2 27
//#define step3 33
#define step0 7
#define step1 8
#define step2 9
#define step3 10
#define COUNT_LOW 0
#define COUNT_HIGH 8888
#include <Pixy2.h>
#include <Servo.h>

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;


// This is the main Pixy object 
Pixy2 pixy;

int freq = 50;
int timeDelay = 100 ;
int timeDelay2 = 1000;

int low = 13;
int med = 17;
int high = 21;

int xCoordinate = 5;
int yCoordinate = 5;
float divNumX = 107;
float divNumY = 67;
int pos = 30;
int pos2x = 30;
int counter = 0;
int token = 0;

int servo0pos = 90;
int servo1pos = 60;
int servo2pos = 90;
int servo3pos = 90;

int xCoordinateTip = 150;
int xCoordinateTracking = 150;
int yCoordinateTip = 100;
int yCoordinateTracking = 100;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
//  
//  ledcSetup(0, freq, 8);
//  ledcAttachPin(step0, 0);
//  ledcSetup(1, freq, 8);
//  ledcAttachPin(step1, 1);
//  ledcSetup(2, freq, 8);
//  ledcAttachPin(step2, 2);
//  ledcSetup(3, freq, 8);
//  ledcAttachPin(step3, 3);
//
//  ledcWrite(1, 17);       // sweep servo 1
//  ledcWrite(3, 17);       // sweep servo 1
//  ledcWrite(0, 17);       // sweep servo 1
//  ledcWrite(2, 17);       // sweep servo 1

  servo0.attach(step0);
  servo1.attach(step1);
  servo2.attach(step2);
  servo3.attach(step3); 
  delay(1000);
  
//  //stage 1 up
//  servo2.write(30);
//  //stage 1 right
//  servo3.write(90);
//  //stage 2 left
//  servo0.write(90);
//  //stage 2 down
//  servo1.write(150);
//
//  delay(1000);

//  stage 1 up
  servo2.write(90);
  //stage 1 right
  servo3.write(90);
  //stage 2 left
  servo0.write(90);
  //stage 2 down
  servo1.write(90);
}

void loop() {
  servo2.write(30);
  servo3.write(30);
  for (pos = 70; pos <= 110; pos += 1) { // goes from 0 degrees to 180 degrees
    servo2.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 70; pos <= 110; pos += 1) { // goes from 180 degrees to 0 degrees
    servo3.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
    for (pos = 110; pos >= 70; pos -= 1) { // goes from 0 degrees to 180 degrees
    servo2.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 110; pos >= 70; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo3.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
