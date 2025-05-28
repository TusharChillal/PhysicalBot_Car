My First ROS Project. 
In this Project the main aim is to make a base car which is the base for the building up and adding on senesors to make to autonumus.


### Phase 1: Manual Motor Control

   Objective : Establish basic motor functionality by manually sending PWM signals from the terminal through a ROS 2 node to an Arduino UNO, which drives the motors via the L298N H-bridge motor driver.

   Hardware Used :
   > Raspberry Pi 5 (ROS 2 installed)
   > Arduino UNO
   > L298N H-Bridge motor driver
   > 4x low-powered DC geared motors (2 per side)
   > 12V battery for motor power
   > USB cable for Arduino-RPi connection

   System Flow :
        [User input Termianl thru SSH] ----> [ROS2 manual_controll.py node] ----> [Arduino PWM Decoder] ----> [L298N Motor Driver] ----> [Motors]

   Phase 1 Outcomes :
  > Verified hardware wiring and motor connections
  > Validated serial communication between ROS 2 and Arduino
  > Successfully drove motors with variable PWM signals
  > Confirmed baseline rover movement before adding autonomous logic


