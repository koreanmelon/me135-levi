#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#define I2C_SDA 12
#define I2C_SCL 13

// https://www.youtube.com/watch?v=q-KIpFIbRMk&t=198s
//Board: ESP32 Wrover Module
//Upload Speed: 115200
//Flash Frequency: "40MHz"
//Flash Mode: "QIO"
//Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
//Port: Whateva

// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

TwoWire I2CSensors = TwoWire(0);


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

//  camera_config_t config;
//  config.ledc_channel = LEDC_CHANNEL_0;
//  config.ledc_timer = LEDC_TIMER_0;
//  config.pin_d0 = Y2_GPIO_NUM;
//  config.pin_d1 = Y3_GPIO_NUM;
//  config.pin_d2 = Y4_GPIO_NUM;
//  config.pin_d3 = Y5_GPIO_NUM;
//  config.pin_d4 = Y6_GPIO_NUM;
//  config.pin_d5 = Y7_GPIO_NUM;
//  config.pin_d6 = Y8_GPIO_NUM;
//  config.pin_d7 = Y9_GPIO_NUM;
//  config.pin_xclk = XCLK_GPIO_NUM;
//  config.pin_pclk = PCLK_GPIO_NUM;
//  config.pin_vsync = VSYNC_GPIO_NUM;
//  config.pin_href = HREF_GPIO_NUM;
//  config.pin_sscb_sda = SIOD_GPIO_NUM;
//  config.pin_sscb_scl = SIOC_GPIO_NUM;
//  config.pin_pwdn = PWDN_GPIO_NUM;
//  config.pin_reset = RESET_GPIO_NUM;
//  config.xclk_freq_hz = 20000000;
//  config.pixel_format = PIXFORMAT_JPEG;
}

void loop() {
  // put your main code here, to run repeatedly:
  I2CSensors.beginTransmission( slaveAddress ); // transmit to device #9

  I2CSensors.write(x); // sends x

  I2CSensors.endTransmission(); // stop transmitting
  delay(10000);
}
