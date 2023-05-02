
#include <Wire.h>
#include <Servo.h>
#include "mpu9250.h"
float xmagOffset = 0;
float ymagOffset = 0;
float zmagOffset = 0;
float xmag = 0;
float ymag = 0;
float zmag = 0;

int token1 = 0;
int cmd = 0;

int servoPin1 = 9;
int servoPin2 = 10;
Servo Servo1;
Servo Servo2;
int angle1 = 90;
int angle2 = 90;
int servoTolerance = 2;

/* Mpu9250 object, I2C bus,  0x68 address */
bfs::Mpu9250 imu(&Wire, 0x68);

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  Servo1.write(angle1);
  Servo2.write(angle2);
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
//  imu.calibrateAccelGyro();
//  imu.calibrateMag();

  if (imu.Read()) {

    xmagOffset = imu.mag_x_ut();
    ymagOffset = imu.mag_y_ut();
    zmagOffset = imu.mag_z_ut();
   
  }
}

void loop() {
  /* Check if data read */

  if (Serial.available() > 0) {
    cmd = Serial.read();
    if(cmd == '0') {
      token1 = 0;
    } else if (cmd == '1') {
      token1 = 1;
    }
  }

  if (token1 == 1) {
    xmagOffset = imu.mag_x_ut();
    ymagOffset = imu.mag_y_ut();
    zmagOffset = imu.mag_z_ut();
    token1 = 0;
    angle1 = 90;
    angle2 = 90;
  }
  
  if (imu.Read()) {

    xmag = imu.mag_x_ut() - xmagOffset;
    ymag = imu.mag_y_ut() - ymagOffset;
    zmag = imu.mag_z_ut() - zmagOffset;
    Serial.print("Mag readings: ");
    Serial.print(xmag);
    Serial.print("\t");
    Serial.print(ymag);
    Serial.print("\t");
    Serial.println(zmag);
    
  }

  if (xmag < -servoTolerance) {
    if (angle1 > 0) {
      angle1 -= 1;
    }
    Servo1.write(angle1);
  } else if (xmag > servoTolerance) {
    if (angle1 < 180) {
      angle1 += 1;
    }
    Servo1.write(angle1);
  }

  if (ymag < -servoTolerance) {
    if (angle2 < 180) {
      angle2 += 1;
    }
    Servo2.write(angle2);
  } else if (ymag > servoTolerance) {
    if (angle2 > 0) {
      angle2 -= 1;
    }
    Servo2.write(angle2);
  }
  Serial.print("Servo Angles: ");
  Serial.print(angle1);
  Serial.print("\t");
  Serial.println(angle2);
  delay(100);
}
