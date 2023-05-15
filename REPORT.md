# ME 135 Project Report: lEVi

Members: Wilder Buchanan, David Guo, Taewon Kim

## Background and Motivation

The main motivation behind this project was to create a device, which we've named lEVi, that can detect a Tesla's charging port and guide a charging cable to it. This would be useful for Tesla owners so that they wouldn't have to leave their car to charge it, whether that's at home in their garage or outside in a designated EV charger.

## Components

-   [OAK-D Lite](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-lite-1)
-   [ESP32 Feather](https://www.adafruit.com/product/3405)
-   [Arduino Mega](https://store-usa.arduino.cc/products/arduino-mega-2560-rev3?selectedStore=us)

## GUI

Given the structure of the various components in our project, the GUI is essentially the central hub of our project. It is responsible for displaying live data about our device, controlling the device, and communicating with the device. The GUI, written in LabVIEW, displays live data about the device, such as the camera's input, the object tracker's output, and the device's current state. In addition, the GUI is able to control the device by sending messages to the device. For example, the GUI is able to send messages to the device to start/stop the device, move the device to a specific position, and move the device to the charging port.

The GUI provides manual adjustment capabilities outside of the object tracking and automatic features. This alone is enough to verify and validate the realtime and multitasking components of this project. Additionally, its ability to control the device's motors and receive live data from the camera provides a useful interface for the user to interact with the device.

## Realtime

This project's realtime component mainly comes from LabVIEW's ability to execute code in realtime. LabVIEW is able to take in data from the camera and process it in realtime. This is important because we want to be able to detect the Tesla's charging port and guide the charging cable to it as quickly as possible. If the device were to take too long to detect and track the charging port, there may be issues in the control algorithm to move the charger to the correct position. Although this component was not part of our project's scope, the device eventually needs to be able to react to the user's actions in realtime. For example, if the user were to move the car while the device is trying to guide the cable to the charging port or some other unexpected event were to occur, the device needs to be able to react to that event in realtime.

## Multitasking

As we already know, multitasking refers to the ability of our device to run multiple tasks concurrently. In our case, we have three tasks that we want to run concurrently: displaying the GUI, retrieving camera inputs, processing camera inputs, and controlling the motors. The host machine executes LabVIEW for the GUI and is responsible for displaying live data about our device. Meanwhile, the camera performs realtime object detection/tracking and transmits its data to the host machine. From there, the host machine is able to send out information to the motor microcontrollers to provide whatever inputs are necessary to perform the intended task. In addition, we've included functionality to the host machine's GUI to control lEVi manually. Device actuation/control happens through an Arduino Mega communicating with the host by proxy of an ESP32-Feather using serial communication. From there, the ESP32 sends and receives messages from the host machine.

These tasks all run simultaneously, minimizing any reason that any one of them may block another's required tasks. For example, the camera is able to perform realtime object detection/tracking without waiting for any inputs or outputs from the host machine's GUI. Additionally, the device itself can continue with its queued processes even if the host is blocked performing some other time-intensive task.

## Conclusion

Over the course of this project's development cycle, we've struggled through various issues with communication and LabVIEW integration. Given what we know now about our project, we'd like to change a whole array of things that would make our lives easier. The limitations of our device comes from the limitations of the OAK-D Lite and the communication process between the various microcontrollers. With a more standalone capable camera, this project would be able to be more independent of the host machine. Additionally, we would like to have a more robust communication protocol between the host machine and the device. This would allow us to have more control over the device and provide more information to the user.

\<Finish the conclusion>