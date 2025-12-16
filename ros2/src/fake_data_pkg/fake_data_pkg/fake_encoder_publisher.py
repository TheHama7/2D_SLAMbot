#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math
import time

class FakeEncoderPublisher(Node):
    def __init__(self):
        super().__init__('fake_encoder_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_encoders)
        self.start_time = time.time()
        self.get_logger().info('Fake Encoder Publisher started.')

    def publish_fake_encoders(self):
        t = time.time() - self.start_time
        # simulate encoder counts
        left_ticks = int(1000 * math.sin(t))
        right_ticks = int(1000 * math.cos(t))

        msg = Int32MultiArray()
        msg.data = [left_ticks, right_ticks]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
