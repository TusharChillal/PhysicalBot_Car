#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class JoyMotorDriver(Node):
    def __init__(self):
        super().__init__('joy_motor_driver')
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to ESP32 via /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.turbo_mode = False
        self.get_logger().info("Subscribed to /joy")

    def joy_callback(self, msg):
        linear_input = msg.axes[1]  # Forward/backward (left stick Y)
        angular_input = msg.axes[2]  # Turning (right stick X)
        button_a = msg.buttons[0]  # A button (Stop)
        button_x = msg.buttons[2]  # X button (Turbo)

        if button_a:
            self.send_pwm(0, 0)
            self.get_logger().info("Emergency Stop (A button pressed)")
            return

        self.turbo_mode = button_x == 1

        linear_gain = 0.6 if not self.turbo_mode else 1.0
        angular_gain = 0.4 if not self.turbo_mode else 0.7

        # Differential drive formula
        left_speed = linear_input - angular_input * angular_gain
        right_speed = linear_input + angular_input * angular_gain

        # Scale to PWM
        scale = 255
        left_pwm = int(max(-255, min(255, left_speed * scale * linear_gain)))
        right_pwm = int(max(-255, min(255, right_speed * scale * linear_gain)))

        self.get_logger().info(
            f"{'TURBO' if self.turbo_mode else 'NORMAL'} | "
            f"Left PWM: {left_pwm}, Right PWM: {right_pwm}"
        )

        self.send_pwm(left_pwm, right_pwm)

    def send_pwm(self, left, right):
        if self.ser.is_open:
            command = f"v {left} {right}\n"
            self.ser.write(command.encode('ascii'))
            self.get_logger().info(f"Sent to ESP32: {command.strip()}")
        else:
            self.get_logger().error("Serial port not open!")

def main(args=None):
    rclpy.init(args=args)
    node = JoyMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
