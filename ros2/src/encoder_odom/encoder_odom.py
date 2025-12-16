#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

WHEEL_RADIUS = 0.033   # meters (example: 66mm diameter wheel -> 0.033 radius)
TRACK_WIDTH  = 0.20    # meters (distance between wheel centers)

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')
        self.sub = self.create_subscription(Float32MultiArray, '/wheel/rpm', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.last_t = time.time()

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

    def cb(self, msg):
        t = time.time()
        dt = t - self.last_t
        if dt <= 0.0: return
        self.last_t = t

        try:
            rpm_l, rpm_r = msg.data[0], msg.data[1]
        except Exception:
            return

        # RPM -> rad/s -> linear wheel speed
        wl = (rpm_l / 60.0) * 2.0 * math.pi
        wr = (rpm_r / 60.0) * 2.0 * math.pi
        vl = wl * WHEEL_RADIUS
        vr = wr * WHEEL_RADIUS

        v = (vr + vl) * 0.5
        w = (vr - vl) / TRACK_WIDTH

        # integrate
        self.yaw += w * dt
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom.twist.twist.linear.x  = v
        self.odom.twist.twist.angular.z = w

        # Basic covariances (tune!): more trust in yaw from IMU, moderate trust in wheel v
        self.odom.pose.covariance = [0.05,0,0, 0,0,0,
                                     0,0.05,0, 0,0,0,
                                     0,0,1e6, 0,0,0,
                                     0,0,0, 1e6,0,0,
                                     0,0,0, 0,1e6,0,
                                     0,0,0, 0,0,0.1]
        self.odom.twist.covariance = [0.1,0,0,0,0,0,
                                      0,0.1,0,0,0,0,
                                      0,0,1e6,0,0,0,
                                      0,0,0,1e6,0,0,
                                      0,0,0,0,1e6,0,
                                      0,0,0,0,0,0.2]

        self.pub.publish(self.odom)

def main():
    rclpy.init()
    rclpy.spin(EncoderOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
