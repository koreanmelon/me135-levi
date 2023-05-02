#include "MPU9250.h"
#include <Servo.h>

Servo servo0;
Servo servo1;

#define step0 9
#define step1 10

MPU9250 mpu;
int pitch = 0;
int yaw = 0;
int roll = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    servo0.attach(step0);
    servo1.attach(step1);
    delay(2000);
    servo0.write(90);
    //stage 2 down
    servo1.write(90);

    
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
            pitch = mpu.getPitch();
            yaw = mpu.getYaw();
            roll = mpu.getRoll();
        }
    }

    
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}
