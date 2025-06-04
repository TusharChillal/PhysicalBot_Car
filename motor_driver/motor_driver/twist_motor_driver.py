#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistMotorDriver(Node):
    def __init__(self):
        super().__init__('twist_motor_driver')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("Connected to ESP32 over serial")

        self.twistSubscription_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.Twist_callback,
            10
        )

    def Twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        wheel_base = 0.13
        left_speed = linear_x - angular_z * wheel_base / 2
        right_speed = linear_x + angular_z * wheel_base / 2
        
        scale = 255 
        left_pwm = int(max(-255, min(255, left_speed * scale)))
        right_pwm = int(max(-255, min(255, right_speed * scale)))   

        self.send_pwm(left_pwm, right_pwm)

    def send_pwm(self, left: int, right: int):
        # Convert signed PWM to direction and magnitude
        def to_dir_speed(pwm):
            direction = 1 if pwm >= 0 else 0  # 1=forward, 0=reverse
            speed = abs(pwm)
            return direction, speed

        left_dir, left_speed = to_dir_speed(left)
        right_dir, right_speed = to_dir_speed(right)

        # Send command in the format:
        # v <left_dir> <left_speed> <right_dir> <right_speed>
        cmd = f"v {left_dir} {left_speed} {right_dir} {right_speed}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = TwistMotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
