#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorTest(Node):
    def __init__(self):
        super().__init__("esp32_motor_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Motor test node started")

    def send_cmd(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.get_logger().info(f"Sending cmd: linear={linear}, angular={angular} ({duration}s)")
        start = time.time()
        rate = self.create_rate(10)

        while time.time() - start < duration:
            self.pub.publish(msg)
            rate.sleep()

    def run_sequence(self):
        # Forward
        self.send_cmd(0.5, 0.0, 2.0)

        # Stop
        self.send_cmd(0.0, 0.0, 1.0)

        # Rotate left
        self.send_cmd(0.0, 0.8, 2.0)

        # Stop
        self.send_cmd(0.0, 0.0, 1.0)

        # Reverse
        self.send_cmd(-0.3, 0.0, 2.0)

        # Stop
        self.send_cmd(0.0, 0.0, 1.0)
        self.get_logger().info("Test sequence complete.")


def main():
    rclpy.init()
    node = MotorTest()

    # Wait a moment for micro-ROS agent + ESP32 connection
    time.sleep(1.0)

    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
