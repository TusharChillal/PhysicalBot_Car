
### Phase 2: Keyboard Motor Control

   Objective : Establish motor functionality by sending speed and direction from the teleop_twist_keyboard through a ROS 2 node to an Arduino UNO, which drives the motors via the L298N H-bridge motor driver.

   Hardware Used :
   > Raspberry Pi 5 (ROS 2 installed) ,
   > ESP32 ,
   > L298N H-Bridge motor driver ,
   > 4x low-powered DC geared motors (2 per side) ,
   > 3x 3.8V li-ion cells,
   > 1x Mini 560 5v Fixed Output Buck (For cooling fan)
   > 1x XL4015 DC to DC Buck Converter (For powering Raspberry Pi 5)
   > USB cable for ESP32-RPi connection ,

   System Flow :
        [User input thru teleop_twist_keyboard] ----> [ROS2 twist_motor_controll.py node] ----> [ESP32 PWM Decoder] ----> [L298N Motor Driver] ----> [Motors]

   Phase 2 Outcomes :
  > Verified hardware wiring and motor connections ,
  > Validated serial communication between ROS 2 and ESP32 ,
  > Successfully drove motors with teleop_twist_keyboard ,


