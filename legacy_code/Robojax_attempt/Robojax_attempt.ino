/*
 * Library: https://github.com/bolderflight/MPU9250
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Updated by Ahmad Shamshiri on July 09, 2018 for Robojax.com
 * in Ajax, Ontario, Canada
 * watch instrucion video for this code: 
For this sketch you need to connect:
VCC to 5V and GND to GND of Arduino
SDA to A4 and SCL to A5

S20A is 3.3V voltage regulator MIC5205-3.3BM5
*/

#include "mpu9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
bfs::Mpu9250 imu(&Wire, 0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with imu 
  status = imu.begin();
  if (status < 0) {
    Serial.println("imu initialization unsuccessful");
    Serial.println("Check imu wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  imu.readSensor();
  // display the data
  Serial.print("AccelX: ");
  Serial.print(imu.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print("AccelY: ");  
  Serial.print(imu.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print("AccelZ: ");  
  Serial.println(imu.getAccelZ_mss(),6);
  
  Serial.print("GyroX: ");
  Serial.print(imu.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print("GyroY: ");  
  Serial.print(imu.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print("GyroZ: ");  
  Serial.println(imu.getGyroZ_rads(),6);

  Serial.print("MagX: ");  
  Serial.print(imu.getMagX_uT(),6);
  Serial.print("\t");  
  Serial.print("MagY: ");
  Serial.print(imu.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print("MagZ: ");  
  Serial.println(imu.getMagZ_uT(),6);
  
  Serial.print("Temperature in C: ");
  Serial.println(imu.getTemperature_C(),6);
  Serial.println();
  delay(200);
} 
