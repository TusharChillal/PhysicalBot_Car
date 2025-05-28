#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node

class ManualMotorDriver(Node):
    def __init__(self):
        super().__init__('manual_motor_driver')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino over serial")

        self.manual_control_loop()

    def manual_control_loop(self):
        try:
            while rclpy.ok():
                left = float(input("Enter left PWM (-255 to 255): "))
                right = float(input("Enter right PWM (-255 to 255): "))
                self.send_pwm(left, right)
        except KeyboardInterrupt:
            self.get_logger().info("Manual control stopped")

    def send_pwm(self, left: float, right: float):
        # Clamp values between -255 and 255
        left = max(-255, min(255, left))
        right = max(-255, min(255, right))
        
        cmd = f"v {left:.0f} {right:.0f}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualMotorDriver()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
