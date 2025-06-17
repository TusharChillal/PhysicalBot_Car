#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class JoyMotorDriver(Node):
    def __init__(self):
        super().__init__('joy_motor_driver_servo')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to ESP32 via /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.update_servo)  # 10 Hz

        self.turbo_mode = False
        self.turn = 90     # Servo 1 position (D-pad left/right)
        self.turn_u = 90   # Servo 2 position (D-pad up/down)
        self.latest_pwm = (0, 0)

        self.dpad_h = 0
        self.dpad_v = 0

        self.last_update_time = time.time()

        self.get_logger().info("Node initialized and subscribed to /joy")

    def joy_callback(self, msg):
        linear_input = msg.axes[1]
        angular_input = msg.axes[2]
        button_a = msg.buttons[0]
        button_x = msg.buttons[2]

        self.dpad_h = msg.axes[6]
        self.dpad_v = msg.axes[7]

        if button_a:
            self.send_pwm(0, 0)
            self.get_logger().info("Emergency Stop (A button pressed)")
            return

        self.turbo_mode = button_x == 1

        linear_gain = 1.0 if self.turbo_mode else 0.6
        angular_gain = 0.7 if self.turbo_mode else 0.4

        left_speed = linear_input - angular_input * angular_gain
        right_speed = linear_input + angular_input * angular_gain

        scale = 255
        left_pwm = int(max(-255, min(255, left_speed * scale * linear_gain)))
        right_pwm = int(max(-255, min(255, right_speed * scale * linear_gain)))

        self.latest_pwm = (left_pwm, right_pwm)

    def update_servo(self):
        # Update every 0.1 seconds (adjust rate for speed)
        current_time = time.time()
        if current_time - self.last_update_time >= 0.1:
            if self.dpad_h == 1:
                self.turn = min(180, self.turn + 10)
            elif self.dpad_h == -1:
                self.turn = max(0, self.turn - 10)

            if self.dpad_v == 1:
                self.turn_u = min(180, self.turn_u + 10)
            elif self.dpad_v == -1:
                self.turn_u = max(0, self.turn_u - 10)

            self.last_update_time = current_time

        # Always send the latest PWM + servo positions
        left_pwm, right_pwm = self.latest_pwm
        self.send_pwm(left_pwm, right_pwm)

    def send_pwm(self, left, right):
        if self.ser.is_open:
            command = f"v {left} {right} {self.turn} {self.turn_u}\n"
            self.ser.write(command.encode('ascii'))
            self.get_logger().info(f"Sent: {command.strip()}")
        else:
            self.get_logger().error("Serial port not open!")

def main(args=None):
    rclpy.init(args=args)
    node = JoyMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
