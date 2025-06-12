Project Overview

      This project documents the step-by-step development of a robust ROS2-powered rover, designed as a modular platform for exploring autonomous navigation and robotic perception. Starting 
      with fundamental motor control and serial communication, the project progressively integrates advanced power management, computational hardware, and eventually, sensor-based autonomy.
![Uploading IMG_5764.jpg…]()

Key Features:

    Modular Development: Built in distinct phases, allowing for iterative testing and integration.
    ROS2-Native Control: Leveraging the latest ROS2 framework for robust and scalable robotic applications.
    Optimized Power Management: Custom power solutions for efficient and reliable operation of all onboard electronics.
    Scalable Architecture: Designed to easily integrate various sensors (Lidar, cameras, IMUs) for future autonomous capabilities.


 Project Phases & Development Branches:

    The project is structured into distinct phases, each managed within its own Git branch. This approach ensures modular development, easier debugging, and a clear progression of features.

   Phase 1: Serial Communication with Arduino (Base)
  
             Establishes fundamental serial communication between a ROS2 node on the host PC and an Arduino UNO controlling the PWM of two DC motors. 
             This phase ensures the basic control loop and hardware interfacing are functional.    

   Phase 2: ROS2 Twist_Keyboard Integration & Power System Upgrade
  
             Integrates the teleop_twist_keyboard package for differential drive control via keyboard input. This phase also includes significant power system upgrades: a 3S Li-ion battery pack with a 
             BMS, a dedicated XL4015 buck converter for the Raspberry Pi 5, and a Mini560 for fan and future servo power, ensuring robust and stable operation. |

   Phase 3 : ROS2 Twist_Joy Integration

              Integrates the teleop_twist_joy package for differnteial drive control via a gamepad or a joystick.
            

 Prerequisites

    ROS2 (Jazzy): Ensure you have a compatible ROS2 distribution installed on your development machine (Phase 1) and Raspberry Pi 5 (Phase 2 onwards).
    Arduino IDE: For uploading code to the Arduino UNO/ESP32.
    Python 3: With pip for installing pyserial and other necessary libraries.
    Basic Linux/Ubuntu Knowledge: Familiarity with command line operations.
    
 Hardware Components

    Arduino UNO/ESP32 (Skip UNO because of less pin use esp32 or STM32)
    DC Geared Motors 
    Motor Driver (L298N) // if possible use a mosfet drivern motor Driver
    3x 18650 Li-ion Cells (2800mAh)
    3S BMS (Battery Management System)
    XL4015 Buck Converter (Step-down Module)
    Mini560 Buck Converter (Step-down Module)
    Raspberry Pi 5
    40mm 5V DC Fan
    2x Servo Motors (for future camera movement)

