#define step1 13
#define step2 12
#define step3 27
#define step4 33
#define COUNT_LOW 0
#define COUNT_HIGH 8888

int freq = 50;
int timeDelay = 750;

void setup() {
  // put your setup code here, to run once:
  ledcSetup(0, freq, 8);
  ledcAttachPin(step1, 0);
  ledcSetup(1, freq, 8);
  ledcAttachPin(step2, 1);
  ledcSetup(2, freq, 8);
  ledcAttachPin(step3, 2);
  ledcSetup(3, freq, 8);
  ledcAttachPin(step4, 3);

  ledcWrite(1, 17);       // sweep servo 1
  ledcWrite(3, 17);       // sweep servo 1
  ledcWrite(0, 17);       // sweep servo 1
  ledcWrite(2, 17);       // sweep servo 1
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:

//  ledcWrite(0, 6);       // sweep servo 1
//  delay(2000);
//  ledcWrite(0, 17);       // sweep servo 1
//  delay(1000)
//  ledcWrite(0, 28);       // sweep servo 1
//  delay(1000);  
//  ledcWrite(0, 17);       // sweep servo 1
//  ledcWrite(1, 17);       // sweep servo 1
//  ledcWrite(2, 17);       // sweep servo 1
//  ledcWrite(3, 17);       // sweep servo 1
//  delay(3000);


  //DEMO below HERE
  ledcWrite(0, 23);       // sweep servo 1
  ledcWrite(2, 23);       // sweep servo 1
  ledcWrite(1, 23);       // sweep servo 1
  ledcWrite(3, 23);       // sweep servo 1
  delay(timeDelay);
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(3, 11);       // sweep servo 1
  delay(timeDelay);

  ledcWrite(0, 11);       // sweep servo 1
  ledcWrite(2, 11);       // sweep servo 1
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(3, 11);       // sweep servo 1
  delay(timeDelay);
  ledcWrite(1, 23);       // sweep servo 1
  ledcWrite(3, 23);       // sweep servo 1
  delay(timeDelay);
  
  ledcWrite(0, 11);       // sweep servo 1
  ledcWrite(2, 23);       // sweep servo 1
  ledcWrite(1, 23);       // sweep servo 1
  ledcWrite(3, 11);       // sweep servo 1
  delay(timeDelay);
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(3, 23);       // sweep servo 1
  delay(timeDelay);

  ledcWrite(0, 23);       // sweep servo 1
  ledcWrite(2, 11);       // sweep servo 1
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(3, 23);       // sweep servo 1
  delay(timeDelay);
  ledcWrite(1, 23);       // sweep servo 1
  ledcWrite(3, 11);       // sweep servo 1
  delay(timeDelay);
  //DEMO above HERE



  
//  delay(3000);
//  ledcWrite(1, 6);       // sweep servo 1
//  ledcWrite(3, 28);       // sweep servo 1
//  delay(3000);
//  ledcWrite(1, 28);       // sweep servo 1
//  ledcWrite(3, 28);       // sweep servo 1
//  delay(3000);
//  ledcWrite(1, 28);       // sweep servo 1
//  ledcWrite(3, 6);       // sweep servo 1
//  delay(3000);

//
//
//  delay(5000);
//  ledcWrite(0, 28);       // sweep servo 1
//  ledcWrite(1, 28);       // sweep servo 1
//  ledcWrite(2, 28);       // sweep servo 1
//  ledcWrite(3, 28);       // sweep servo 1
//  delay(5000);
//  ledcWrite(0, 21);       // sweep servo 1
//  ledcWrite(1, 21);       // sweep servo 1
//  ledcWrite(2, 21);       // sweep servo 1
//  ledcWrite(3, 21);       // sweep servo 1
//  delay(2000);
//  ledcWrite(0, 23);       // sweep servo 1
//  ledcWrite(1, 23);       // sweep servo 1
//  ledcWrite(2, 23);       // sweep servo 1
//  ledcWrite(3, 23);       // sweep servo 1
//  delay(2000);
//  

//  delay(1000);
//  ledcWrite(2, 28);       // sweep servo 1
//  delay(2000);
//  ledcWrite(2, 6);       // sweep servo 1
//  delay(2000);
//  ledcWrite(2, 17);       // sweep servo 1
}


//
//#include <Servo.h>
//
//#define SERVO_PIN 13 // ESP32 pin GIOP26 connected to servo motor
//
//Servo servoMotor;
//
//void setup() {
//  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
//}
//
//void loop() {
//  // rotates from 0 degrees to 180 degrees
//  for (int pos = 0; pos <= 180; pos += 1) {
//    // in steps of 1 degree
//    servoMotor.write(pos);
//    delay(15); // waits 15ms to reach the position
//  }
//
//  // rotates from 180 degrees to 0 degrees
//  for (int pos = 180; pos >= 0; pos -= 1) {
//    servoMotor.write(pos);
//    delay(15); // waits 15ms to reach the position
//  }
//}
