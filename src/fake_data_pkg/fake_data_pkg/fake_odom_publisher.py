#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import math
import time

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_odom)  # 10 Hz
        self.start_time = time.time()
        self.get_logger().info('Fake Odom Publisher started.')

    def publish_fake_odom(self):
        t = time.time() - self.start_time
        # simple circular motion
        x = math.cos(t) * 1.0
        y = math.sin(t) * 1.0
        yaw = t % (2 * math.pi)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )
        odom_msg.twist.twist.linear.x = -math.sin(t)
        odom_msg.twist.twist.linear.y = math.cos(t)
        odom_msg.twist.twist.angular.z = 1.0

        self.publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
