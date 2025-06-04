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
            10 # QoS history depth
        )
        self.get_logger().info("Subscribed to /cmd_vel topic")

    def Twist_callback(self, msg):

        linear_x = msg.linear.x    # Linear velocity in x-direction (forward/backward)
        angular_z = msg.angular.z  # Angular velocity around z-axis (turning)

        wheel_base = 0.13 # meters

        left_speed = linear_x - angular_z * wheel_base / 2
        right_speed = linear_x + angular_z * wheel_base / 2
        
        scale = 255 

        left_pwm = int(max(-255, min(255, left_speed * scale)))
        right_pwm = int(max(-255, min(255, right_speed * scale)))   

        self.get_logger().info(f"Calculated PWMs: Left={left_pwm}, Right={right_pwm}")


        self.send_pwm(left_pwm, right_pwm)

    def send_pwm(self, left_pwm_val: int, right_pwm_val: int):
        cmd = f"v {left_pwm_val} {right_pwm_val}\n"
        self.ser.write(cmd.encode('ascii'))
        self.get_logger().info(f"Sent to ESP32: '{cmd.strip()}'")


def main(args=None):
    rclpy.init(args=args)
    node = TwistMotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()