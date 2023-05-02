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
int token = 1;

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
  servo1.write(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  //  for (pos = 30; pos <= 150; pos += 1) { // goes from 0 degrees to 180 degrees
  //    // in steps of 1 degree
  //    pos2x += 2;
  //    servo0.write(pos2x);              // tell servo to go to position in variable 'pos'
  //    servo1.write(180-pos);
  //    servo2.write(pos);
  //    servo3.write(pos2x);
  //    delay(15);                       // waits 15ms for the servo to reach the position
  //  }
  //  for (pos = 150; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
  //    servo0.write(pos);              // tell servo to go to position in variable 'pos'
  //    servo1.write(180-pos);              // tell servo to go to position in variable 'pos'
  //    servo2.write(pos);
  //    servo3.write(pos2x);
  //    delay(15);                       // waits 15ms for the servo to reach the position
  //  }
  //}
  if (token == 0) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
      xCoordinate = map(pixy.ccc.blocks[0].m_x, 0, 320, 30, 150);
      yCoordinate = map(pixy.ccc.blocks[0].m_y, 0, 200, 30, 150);
      token = 2;
      servo1.write(60);
      servo2.write(90);

    }
  }

  if (token == 1) {
    for (pos = 150; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo0.write(pos);              // tell servo to go to position in variable 'pos'
      servo3.write(pos);              // tell servo to go to position in variable 'pos'

      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 30; pos <= 150; pos += 1) { // goes from 180 degrees to 0 degrees
      servo3.write(pos);              // tell servo to go to position in variable 'pos'
      servo0.write(pos);              // tell servo to go to position in variable 'pos'

      delay(15);                       // waits 15ms for the servo to reach the position
    }


    //    servo2.write(90);
    //    servo3.write(150);
    //    servo0.write(150);
    //    servo1.write(90);
    //    delay(2000);
    token = 1;
  }


  //  if (token == 2) {
  //    pixy.ccc.getBlocks();
  //    xCoordinateTip = pixy.ccc.blocks[0].m_x;
  //    yCoordinateTip = pixy.ccc.blocks[0].m_y;
  //    if (pixy.ccc.numBlocks > 1) {
  //      yCoordinateTracking = pixy.ccc.blocks[1].m_y;
  //      xCoordinateTracking = pixy.ccc.blocks[1].m_x;
  //    }

  Serial.println("xCoordinateTip, yCoordinateTip, xCoordinateTracking, yCoordinateTracking");
  Serial.print(xCoordinateTip);
  Serial.print(", ");
  Serial.print(yCoordinateTip);
  Serial.print(", ");
  Serial.print(xCoordinateTracking);
  Serial.print(", ");
  Serial.println(yCoordinateTracking);


  if (xCoordinateTip < xCoordinateTracking) {
    servo0pos = servo0pos - 2;
    servo0.write(servo0pos);
  }
  if (xCoordinateTip > xCoordinateTracking) {
    servo0pos = servo0pos + 2;
    servo0.write(servo0pos);
  }
  if (yCoordinateTip < yCoordinateTracking) {
    servo1pos = servo1pos + 2;
    servo1.write(servo1pos);
  }
  if (yCoordinateTip > yCoordinateTracking) {
    servo1pos = servo1pos - 2;
    servo1.write(servo1pos);
  }
  delay(100);
}



//
//    //  stage 1 up
//    servo2.write(90);
//    //stage 1 right
//    servo3.write(90);
//    //stage 2 left
//    servo0.write(90);
//    //stage 2 down
//    servo1.write(60);
//
