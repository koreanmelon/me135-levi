#include <Wire.h>
#include <Servo.h>
// Serial Read input
String input;

//Button Setup
const int buttonPin = 11;
int buttonState = 0;
int anglechangex = 0;
int anglechangey = 0;



int servoPin1 = 9;
int servoPin2 = 10;
Servo Servo1;
Servo Servo2;

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error = 0;                       //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;    //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

int Total_angle_x, Total_angle_y;



void setup() {

  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  Servo1.write(90);
  Servo2.write(90);
  Serial.println("Servos Initialized");
  Wire.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  //Gyro config
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  //  //Acc config
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission(true);

  Serial.begin(9600);
  time = millis();



  if (acc_error == 0)
  {
    for (int a = 0; a < 200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

      //
      //      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));

      if (a == 199)
      {
        Acc_angle_error_x = Acc_angle_error_x / 200;
        Acc_angle_error_y = Acc_angle_error_y / 200;
        acc_error = 1;
      }
    }
  }//end of acc error calculation



  if (gyro_error == 0)
  {
    for (int i = 0; i < 200; i++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 4, true);

      Gyr_rawX = Wire.read() << 8 | Wire.read();
      Gyr_rawY = Wire.read() << 8 | Wire.read();

      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
      /*---Y---*/



      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);

      \
      if (i == 199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 200;
        Gyro_raw_error_y = Gyro_raw_error_y / 200;
        gyro_error = 1;
      }
    }
  }//end of gyro error calculation
  //Button Setup

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

}//end of setup void


void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;
  //////////////////////////////////////Gyro read/////////////////////////////////////

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  /*---X---*/
  Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  /*---Y---*/
  Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y; //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)


  /*---X---*/
  Gyro_angle_x = Gyr_rawX * elapsedTime;
  /*---X---*/
  Gyro_angle_y = Gyr_rawY * elapsedTime;




  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

  /*---X---*/
  Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
  /*---Y---*/
  Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;


  //////////////////////////////////////Total angle and filter/////////////////////////////////////
  //  /*---X axis angle---*/
  Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
  /*---Y axis angle---*/
  Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;
  //  //
//  int Tot_angle_x,Tot_angle_y;
//  //Total angle calcuations
//  Tot_angle_x = (Tot_angle_x + Gyro_angle_x);
//  //  // /*---Y axis angle---*/
//  Tot_angle_y = (Tot_angle_y + Gyro_angle_y);
//  //  //
  //Total_angle_x =  Total_angle_x + (Gyr_rawX/ 32.8);
  //Total_angle_y = Total_angle_y + (Gyr_rawY/ 32.8);


  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    Serial.println("Button Pressed");
    anglechangex = Total_angle_x;
    anglechangey = Total_angle_y;
  }
  
  // Adding offset
  float finalangx, finalangy;

  if (anglechangex > 0) {
    finalangx = Total_angle_x + 90 - anglechangex;
  }
  else if (anglechangex < 0) {
    finalangx = Total_angle_x + 90 + anglechangex;
  }
  if (anglechangey > 0) {
    finalangy = Total_angle_y + 90 - anglechangey;
  }
  else if (anglechangey < 0) {
    finalangy = Total_angle_y + 90 + anglechangey;
  }
  else {
    finalangx = (-3*Total_angle_x) + 85;
    finalangy = (-1.5*Total_angle_y) + 80;
  }

  //  else {
  //    Servo1.write(Total_angle_x + 100);
  //    Servo2.write(Total_angle_y + 100);
  //  }

  // Failsafe for high offset values
  if (finalangx > 180 || finalangx < 0) {
    finalangx = Total_angle_x + 90;
  }
  if (finalangy > 180 || finalangy < 0) {
    finalangy = Total_angle_y + 90;
  }

  
  Servo1.write(finalangx);
  Servo2.write(finalangy);
//  Serial.print("Xº: ");
//  Serial.print(Tot_angle_x);
//  Serial.print("   ");
//  Serial.print("Yº: ");
//  Serial.println(Tot_angle_y);

  Serial.print("Xº: ");
  Serial.print(Total_angle_x);
  Serial.print("   ");
  Serial.print("Yº: ");
  Serial.print(Total_angle_y);
  Serial.print(" ");
//  Serial.print("Offset X: ");
//  Serial.print(anglechangex);
//  Serial.print("   ");
//  Serial.print("Offset Y: ");
//  Serial.print(anglechangey);
  Serial.print(" ");
  Serial.print("Fin Ang X: ");
  Serial.print(finalangx);
  Serial.print("   ");
  Serial.print("Fin Ang Y: ");
  Serial.print(finalangy);
  Serial.println(" ");


}
