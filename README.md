
### Phase 2: Joystick Motor Control

   Objective : Establish motor functionality by sending speed and direction from the a Joystrick/gamepad through a ROS 2 node to an ESP32, which drives the motors via the L298N H-bridge motor driver.

   Hardware Used :
   > Raspberry Pi 5 (ROS 2 installed) ,
   > ESP32 ,
   > L298N H-Bridge motor driver ,
   > 4x low-powered DC geared motors (2 per side) ,
   > 3x 3.8V li-ion cells,
   > 1x Mini 560 5v Fixed Output Buck (For cooling fan)
   > 1x XL4015 DC to DC Buck Converter (For powering Raspberry Pi 5)
   > USB cable for ESP32-RPi connection ,
   > Joystick/Gamepad

   System Flow :
        [User input thru Joystick/Gamepad] ----> [ROS2 joy_motor_driver.py node] ----> [ESP32 PWM Decoder] ----> [L298N Motor Driver] ----> [Motors]

   Phase 2 Outcomes :
  > Verified hardware wiring and motor connections ,
  > Validated serial communication between ROS 2 and ESP32 ,
  > Successfully drove motors with a Joystrick/Gamepad ,


