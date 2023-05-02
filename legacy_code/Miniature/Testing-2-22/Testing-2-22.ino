#define step0 13
#define step1 12
#define step2 27
#define step3 33
#define COUNT_LOW 0
#define COUNT_HIGH 8888

int freq = 50;
int timeDelay = 750;
int timeDelay2 = 1000;

int low = 13;
int med = 17;
int high = 21;

void setup() {
  // put your setup code here, to run once:
  ledcSetup(0, freq, 8);
  ledcAttachPin(step0, 0);
  ledcSetup(1, freq, 8);
  ledcAttachPin(step1, 1);
  ledcSetup(2, freq, 8);
  ledcAttachPin(step2, 2);
  ledcSetup(3, freq, 8);
  ledcAttachPin(step3, 3);

  ledcWrite(1, 17);       // sweep servo 1
  ledcWrite(3, 17);       // sweep servo 1
  ledcWrite(0, 17);       // sweep servo 1
  ledcWrite(2, 17);       // sweep servo 1
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:


  for (int i = 1; i < 10; i++) {
    s1Move(i);
    s2Move(i);
    delay(2000);
  }
//
//  s2Move(4);
//  s1Move(5);
//  delay(2000);
//  s2Move(6);
//  delay(2000);
}

void s1Move(int pos){
  if(pos == 3) {
    ledcWrite(3, high);       // sweep servo 1
    ledcWrite(2, high);       // sweep servo 1
  } else if(pos == 2) {
    ledcWrite(3, med);       // sweep servo 1
    ledcWrite(2, high);  
  } else if(pos == 1) {
    ledcWrite(3, low);       // sweep servo 1
    ledcWrite(2, high);  
  } else if(pos == 6) {
    ledcWrite(3, high);       // sweep servo 1
    ledcWrite(2, med);  
  } else if(pos == 5) {
    ledcWrite(3, med);       // sweep servo 1
    ledcWrite(2, med);  
  } else if(pos == 4) {
    ledcWrite(3, low);       // sweep servo 1
    ledcWrite(2, med);  
  } else if(pos == 9) {
    ledcWrite(3, high);       // sweep servo 1
    ledcWrite(2, low);  
  } else if(pos == 8) {
    ledcWrite(3, med);       // sweep servo 1
    ledcWrite(2, low);  
  } else if(pos == 7) {
    ledcWrite(3, low);       // sweep servo 1
    ledcWrite(2, low);  
  }
}


void s2Move(int pos){
  if(pos == 3) {
    ledcWrite(1, low);       // sweep servo 1
    ledcWrite(0, low);       // sweep servo 1
  } else if(pos == 2) {
    ledcWrite(1, low);       // sweep servo 1
    ledcWrite(0, med);  
  } else if(pos == 1) {
    ledcWrite(1, low);       // sweep servo 1
    ledcWrite(0, high);  
  } else if(pos == 6) {
    ledcWrite(1, med);       // sweep servo 1
    ledcWrite(0, low);  
  } else if(pos == 5) {
    ledcWrite(1, med);       // sweep servo 1
    ledcWrite(0, med);  
  } else if(pos == 4) {
    ledcWrite(1, med);       // sweep servo 1
    ledcWrite(0, high);  
  } else if(pos == 9) {
    ledcWrite(1, high);       // sweep servo 1
    ledcWrite(0, low);  
  } else if(pos == 8) {
    ledcWrite(1, high);       // sweep servo 1
    ledcWrite(0, med);  
  } else if(pos == 7) {
    ledcWrite(1, high);       // sweep servo 1
    ledcWrite(0, high);  
  }
}
