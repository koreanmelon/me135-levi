// CODE MEANT FOR ARDUINO


//#define step0 13
//#define step1 12
//#define step2 27
//#define step3 33
#define step0 5
#define step1 6
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
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (pos = 30; pos <= 150; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pos2x += 2;
    servo0.write(pos2x);              // tell servo to go to position in variable 'pos'
    servo1.write(180-pos);
    servo2.write(pos);
    servo3.write(pos2x);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 150; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo0.write(pos);              // tell servo to go to position in variable 'pos'
    servo1.write(180-pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);
    servo3.write(pos2x);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

//  pixy.ccc.getBlocks();
//  if (pixy.ccc.numBlocks) {
//    xCoordinate = map(pixy.ccc.blocks[0].m_x, 0, 320, 30, 150);
//    yCoordinate = map(pixy.ccc.blocks[0].m_y, 0, 200, 30, 150);
//
//  }
//  servo2.write(180-yCoordinate);
//  servo3.write(xCoordinate);
//  servo0.write(180 - xCoordinate);
//  servo1.write(yCoordinate);
//}
//
////  int i; 
//
//  // grab blocks!
//  pixy.ccc.getBlocks();
//  
//  // If there are detect blocks, print them!
//  if (pixy.ccc.numBlocks)
//  {
////    Serial.print("Detected ");
////    Serial.println(pixy.ccc.numBlocks);
//    for (int i=0; i<pixy.ccc.numBlocks; i++)
//    {
//      Serial.print("  block ");
//      Serial.print(i);
//      Serial.print(": ");
//      //pixy.ccc.blocks[i].print();
//      
//      Serial.print(pixy.ccc.blocks[i].m_x);
//      Serial.print(", ");
//      Serial.println(pixy.ccc.blocks[i].m_y);
//      xCoordinate = round((pixy.ccc.blocks[i].m_x)/divNumX);
//      s1Move(xCoordinate);
//      yCoordinate = round((pixy.ccc.blocks[i].m_y)/divNumY);
//      s2Move(yCoordinate);
//    }
//  }  
//  
//
//  
//  
////  ledcWrite(1, 17);       // sweep servo 1
////  ledcWrite(3, 17);       // sweep servo 1
////  ledcWrite(2, 6);       // sweep servo 1
////  ledcWrite(3, 6);       // sweep servo 1
////  delay(3000);
////  ledcWrite(2, 6);       // sweep servo 1
////  ledcWrite(3, 28);       // sweep servo 1
////  delay(3000);
////  ledcWrite(2, 28);       // sweep servo 1
////  ledcWrite(3, 28);       // sweep servo 1
////  delay(3000);
////  ledcWrite(2, 28);       // sweep servo 1
////  ledcWrite(3, 6);       // sweep servo 1
////  delay(3000);
////  for (int i = 1; i < 10; i++) {
////    s2Move(i);
////    s1Move(i);
////    delay(2000);
////  }
//
//
//
//
//
//
////Generic
////  for (int i = 1; i < 10; i++) {
////    s1Move(i);
////    for (int j = 1; j < 4; j++) {
////      s2Move(j);
////      delay(1000);
////
////    }
////  }
//
//// diagonal
////  s1Move(9);
////  s2Move(9);
////  delay(150);
////  s1Move(9);
////  s2Move(1);
////  delay(150);
////  s1Move(5);
////  s2Move(1);
////  delay(150);
////  s1Move(1);
////  s2Move(1);
////  delay(150);
////  s2Move(9);
////  s1Move(1);
////  delay(150);
////  s2Move(9);
////  s1Move(5);
////  delay(150);
//
////Figure 8
////  s1Move(9);
////  s2Move(9);
////  delay(timeDelay);
////  s1Move(9);
////  s2Move(8);
////  delay(timeDelay);
////  s1Move(8);
////  s2Move(7);
////  delay(timeDelay);
////  s1Move(7);
////  s2Move(7);
////  delay(timeDelay);
////  s1Move(7);
////  s2Move(5);
////  delay(timeDelay);
////  s1Move(5);
////  s2Move(3);
////  delay(timeDelay);
////  s1Move(3);
////  s2Move(3);
////  delay(timeDelay);
////  s1Move(3);
////  s2Move(2);
////  delay(timeDelay);
////  s1Move(2);
////  s2Move(1);
////  delay(timeDelay);
////  s1Move(1);
////  s2Move(1);
////  delay(timeDelay);
////  s1Move(1);
////  s2Move(5);
////  delay(timeDelay);
////  s1Move(5);
////  s2Move(9);
////  delay(timeDelay);
////  s1Move(9);
////  s2Move(9);
////  delay(timeDelay);
//
//
////  s1Move(1);
////  s2Move(2);
////  delay(2000);
//}
//
//void s1Move(int pos){
//  if(pos == 3) {
//    ledcWrite(3, high);       // sweep servo 1
//    ledcWrite(2, high);       // sweep servo 1
//  } else if(pos == 2) {
//    ledcWrite(3, med);       // sweep servo 1
//    ledcWrite(2, high);  
//  } else if(pos == 1) {
//    ledcWrite(3, low);       // sweep servo 1
//    ledcWrite(2, high);  
//  } else if(pos == 6) {
//    ledcWrite(3, high);       // sweep servo 1
//    ledcWrite(2, med);  
//  } else if(pos == 5) {
//    ledcWrite(3, med);       // sweep servo 1
//    ledcWrite(2, med);  
//  } else if(pos == 4) {
//    ledcWrite(3, low);       // sweep servo 1
//    ledcWrite(2, med);  
//  } else if(pos == 9) {
//    ledcWrite(3, high);       // sweep servo 1
//    ledcWrite(2, low);  
//  } else if(pos == 8) {
//    ledcWrite(3, med);       // sweep servo 1
//    ledcWrite(2, low);  
//  } else if(pos == 7) {
//    ledcWrite(3, low);       // sweep servo 1
//    ledcWrite(2, low);  
//  }
//}
//
//
//void s2Move(int pos){
//  if(pos == 3) {
//    ledcWrite(1, low);       // sweep servo 1
//    ledcWrite(0, low);       // sweep servo 1
//  } else if(pos == 2) {
//    ledcWrite(1, low);       // sweep servo 1
//    ledcWrite(0, med);  
//  } else if(pos == 1) {
//    ledcWrite(1, low);       // sweep servo 1
//    ledcWrite(0, high);  
//  } else if(pos == 6) {
//    ledcWrite(1, med);       // sweep servo 1
//    ledcWrite(0, low);  
//  } else if(pos == 5) {
//    ledcWrite(1, med);       // sweep servo 1
//    ledcWrite(0, med);  
//  } else if(pos == 4) {
//    ledcWrite(1, med);       // sweep servo 1
//    ledcWrite(0, high);  
//  } else if(pos == 9) {
//    ledcWrite(1, high);       // sweep servo 1
//    ledcWrite(0, low);  
//  } else if(pos == 8) {
//    ledcWrite(1, high);       // sweep servo 1
//    ledcWrite(0, med);  
//  } else if(pos == 7) {
//    ledcWrite(1, high);       // sweep servo 1
//    ledcWrite(0, high);  
//  }
//}
