#include <MPU9250_WE.h>
#include <Servo.h>
#include <SpeedyStepper.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68

float tol = 1;

String inputString = "";     // a String to hold incoming data
bool stringComplete = false; // whether the string is complete

float offsetX, offsetY, offsetZ;
float xValue, yValue, zValue;
float xError, yError, zError;

bool s1udINTERUPT = false;
bool s1lrINTERUPT = false;
bool s2udINTERUPT = false;
bool s2lrINTERUPT = false;

const int BP1 = 44;
const int BP2 = 45;
const int BP3 = 42;
const int BP4 = 43;
const int BP5 = 40;
const int BP6 = 41;
const int BP7 = 38;
const int BP8 = 39;

const int MP1 = 18;
const int MP2 = 2;
const int MP3 = 19;
const int MP4 = 4;
const int MP5 = 20;
const int MP6 = 6;
const int MP7 = 21;
const int MP8 = 8;

int BS1, BS2, BS3, BS4, BS5, BS6, BS7, BS8 = 0;
int MS1, MS2, MS3, MS4, MS5, MS6, MS7, MS8 = 0;

int servoPin1 = 9;
int servoPin2 = 10;
int servoPin3 = 11;

Servo Servo1, Servo2, Servo3;

float angle1 = 110;
float angle2 = 70;
float angle3 = 0;

// Stage 1 up-down
#define dirPin1 34
#define stepPin1 35

// Stage 2 up-down
#define dirPin2 32
#define stepPin2 33

// Stage 1 left-right
#define dirPin3 36
#define stepPin3 37

// Stage 2 left-right
#define dirPin4 30
#define stepPin4 31

int maxSpeedy = 500000;
float maxAccel = 100000;
int stepSize = 500;
int ButtonStepSize = 500;

String ao = "";
bool inputReady = false;

#define motorInterfaceType 1 // Create a new instance of the AccelStepper class:
SpeedyStepper stepper1, stepper2, stepper3, stepper4;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// Interrupt Service Routines

void s1ud()
{
    s1udINTERUPT = true;
}

void s1lr()
{
    s1lrINTERUPT = true;
}
void s2ud()
{
    s2udINTERUPT = true;
}
void s2lr()
{
    s2lrINTERUPT = true;
}

/**
 * @brief Runs the stepper motor until the Hall effect sensor is triggered
 */
void runMotor(SpeedyStepper stepper, int magSwitch, int dir)
{
    while (digitalRead(magSwitch) == HIGH)
    {
        stepper.setupRelativeMoveInSteps(stepSize * dir);
        while (!stepper.motionComplete())
        {
            stepper.processMovement();
        }
    }
}

/**
 * @brief Runs the stepper motor for a given number of steps
 */
void runSteps(SpeedyStepper stepper, int dir)
{
    stepper.setupRelativeMoveInSteps(stepSize * dir);
    while (!stepper.motionComplete())
    {
        stepper.processMovement();
    }
}

void serialEvent2()
{
    while (Serial2.available())
    {
        ao = Serial2.readString();
        inputReady = true;
    }
}

void setup()
{
    Servo1.attach(servoPin1);
    Servo2.attach(servoPin2);
    Servo3.attach(servoPin3);

    Servo1.write(int(angle1));
    Servo2.write(int(angle2));
    Servo3.write(int(angle3));
    delay(1000);

    Serial.begin(115200);
    Serial2.begin(115200);
    while (!Serial)
    {
    }
    while (!Serial2)
    {
    }
    Serial.println("Servos initialized");
    inputString.reserve(200);
    Wire.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    stepper1.connectToPins(stepPin1, dirPin1);
    stepper2.connectToPins(stepPin2, dirPin2);
    stepper3.connectToPins(stepPin3, dirPin3);
    stepper4.connectToPins(stepPin4, dirPin4);

    stepper1.setSpeedInStepsPerSecond(maxSpeedy);
    stepper1.setAccelerationInStepsPerSecondPerSecond(maxAccel);
    stepper2.setSpeedInStepsPerSecond(maxSpeedy);
    stepper2.setAccelerationInStepsPerSecondPerSecond(maxAccel);
    stepper3.setSpeedInStepsPerSecond(maxSpeedy);
    stepper3.setAccelerationInStepsPerSecondPerSecond(maxAccel);
    stepper4.setSpeedInStepsPerSecond(maxSpeedy);
    stepper4.setAccelerationInStepsPerSecondPerSecond(maxAccel);

    pinMode(BP1, INPUT);
    pinMode(BP2, INPUT);
    pinMode(BP3, INPUT);
    pinMode(BP4, INPUT);
    pinMode(BP7, INPUT);
    pinMode(BP8, INPUT);
    pinMode(BP5, INPUT);
    pinMode(BP6, INPUT);

    attachInterrupt(digitalPinToInterrupt(MP1), s1ud, FALLING);
    pinMode(MP2, INPUT);
    attachInterrupt(digitalPinToInterrupt(MP3), s1lr, FALLING);
    pinMode(MP4, INPUT);
    attachInterrupt(digitalPinToInterrupt(MP5), s2ud, FALLING);
    pinMode(MP6, INPUT);
    attachInterrupt(digitalPinToInterrupt(MP7), s2lr, FALLING);
    pinMode(MP8, INPUT);

    Serial.print("Exiting setup...");
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(5000);                     // wait 5 seconds
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
}

void loop()
{
    BS1 = digitalRead(BP1);
    BS2 = digitalRead(BP2);
    BS3 = digitalRead(BP3);
    BS4 = digitalRead(BP4);
    BS5 = digitalRead(BP5);
    BS6 = digitalRead(BP6);
    BS7 = digitalRead(BP7);
    BS8 = digitalRead(BP8);

    if (BS1 == HIGH)
    {
        stepper1.setupRelativeMoveInSteps(4 * ButtonStepSize); // Set the speed in steps per second:
        Serial.print("BS1");
    }
    else if (BS2 == HIGH)
    {
        stepper1.setupRelativeMoveInSteps(-4 * ButtonStepSize); // Set the speed in steps per second:
    }
    if (BS3 == HIGH)
    {
        stepper2.setupRelativeMoveInSteps(ButtonStepSize); // Set the speed in steps per second:
    }
    else if (BS4 == HIGH)
    {
        stepper2.setupRelativeMoveInSteps(-ButtonStepSize); // Set the speed in steps per second:
    }
    if (BS5 == HIGH)
    {
        stepper3.setupRelativeMoveInSteps(ButtonStepSize); // Set the speed in steps per second:
    }
    else if (BS6 == HIGH)
    {
        stepper3.setupRelativeMoveInSteps(-ButtonStepSize); // Set the speed in steps per second:
    }
    if (BS7 == HIGH)
    {
        stepper4.setupRelativeMoveInSteps(-2 * ButtonStepSize); // Set the speed in steps per second:
    }
    else if (BS8 == HIGH)
    {
        stepper4.setupRelativeMoveInSteps(2 * ButtonStepSize); // Set the speed in steps per second:
    }
    while ((!stepper1.motionComplete()) || (!stepper2.motionComplete()) || (!stepper3.motionComplete()) ||
           (!stepper4.motionComplete()))
    {
        stepper1.processMovement();
        stepper2.processMovement();
        stepper3.processMovement();
        stepper4.processMovement();
    }

    if (inputReady)
    {
        Serial.println(ao);
        inputReady = false;
        if (ao == "s1up")
        {
            runSteps(stepper1, 1);
            Serial.println("s1upACTIVE");
        }
        else if (ao == "s1down")
        {
            Serial.println("s1downACTIVE");
            stepper1.setupRelativeMoveInSteps(-500);
            while (!stepper1.motionComplete())
            {
                stepper1.processMovement();
            }
        }
        else if (ao == "s1left")
        {
            runSteps(stepper3, 1);
        }
        else if (ao == "s1right")
        {
            runSteps(stepper3, -1);
        }
        else if (ao == "s2up")
        {
            runSteps(stepper2, 1);
        }
        else if (ao == "s2down")
        {
            runSteps(stepper2, -1);
        }
        else if (ao == "s2left")
        {
            runSteps(stepper4, 1);
        }
        else if (ao == "s2right")
        {
            runSteps(stepper4, -1);
        }
        else if (ao == "home")
        {
            runMotor(stepper3, MP5, 1);
            runMotor(stepper1, MP1, 1);
            runMotor(stepper4, MP7, 1);
            runMotor(stepper2, MP3, 1);

            runMotor(stepper1, MP2, -1);
            runMotor(stepper3, MP6, -1);
            runMotor(stepper2, MP4, -1);
            runMotor(stepper4, MP8, -1);
        }
        else if (ao == "plug")
        {
            Servo3.write(180);
            delay(1000);
        }
        else if (ao == "unplug")
        {
            Servo3.write(0);
            delay(1000);
        }

        ao = "";
    }

    if (s1udINTERUPT)
    {
        s1udINTERUPT = false;
        Serial.println("interupt 1");
        runMotor(stepper1, MP2, -1);
    }
    else if (s1lrINTERUPT)
    {
        s1lrINTERUPT = false;
        Serial.println("interupt 2");
        runMotor(stepper2, MP4, -1);
    }
    else if (s2udINTERUPT)
    {
        s2udINTERUPT = false;
        Serial.println("interupt 3");
        runMotor(stepper3, MP6, -1);
    }
    else if (s2lrINTERUPT)
    {
        s2lrINTERUPT = false;
        Serial.println("interupt 4");
        runMotor(stepper4, MP8, -1);
    }
}
