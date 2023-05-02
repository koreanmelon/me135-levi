#define step1 13
#define step2 12
#define step3 27
#define step4 33
#define COUNT_LOW 0
#define COUNT_HIGH 8888

int freq = 50;
int timeDelay = 750;
int timeDelay2 = 1000;

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
//
//  ledcWrite(0, 17);       // sweep servo 1
//  ledcWrite(1, 26);       // sweep servo 1
//  ledcWrite(2, 8);       // sweep servo 1
//  ledcWrite(3, 17);       // sweep servo 1
  ledcWrite(0, 17);       // sweep servo 1
  ledcWrite(1, 17);       // sweep servo 1
  ledcWrite(2, 17);       // sweep servo 1
  ledcWrite(3, 17);       // sweep servo 1
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:

  //DEMO below HERE
  ledcWrite(0, 23);       // sweep servo 1
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(2, 23);       // sweep servo 1
  ledcWrite(3, 11);       // sweep servo 1
  delay(timeDelay2);
  
  ledcWrite(0, 17);       // sweep servo 1
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(2, 23);       // sweep servo 1
  ledcWrite(3, 17);       // sweep servo 1
  delay(timeDelay2);

  ledcWrite(0, 11);       // sweep servo 1
  ledcWrite(1, 11);       // sweep servo 1
  ledcWrite(2, 23);       // sweep servo 1
  ledcWrite(3, 23);       // sweep servo 1
  delay(timeDelay2);
 
  //DEMO above HERE

}
