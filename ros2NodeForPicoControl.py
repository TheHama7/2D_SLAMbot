# this node sends motor speed commands to pico
#it reads encoder data and publishes to /odom
# it subscribes to /cmd_vel 



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import math
import time

WHEEL_RADIUS = 0.033  # meters
BASE_WIDTH = 0.15     # meters
TICKS_PER_REV = 360   # encoder ticks per revolution

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.prev_left = 0
        self.prev_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.timer = self.create_timer(0.05, self.update_odometry)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        v_l = linear - (angular * BASE_WIDTH / 2)
        v_r = linear + (angular * BASE_WIDTH / 2)
        pwm_l = int(max(min(v_l * 1000, 65535), -65535))
        pwm_r = int(max(min(v_r * 1000, 65535), -65535))
        command = f"SET L_SPEED {pwm_l} R_SPEED {pwm_r}\n"
        self.ser.write(command.encode())

    def update_odometry(self):
        self.ser.write(b"GET ODOM\n")
        line = self.ser.readline().decode().strip()
        if "ENC" in line:
            try:
                parts = line.split()
                left_ticks = int(parts[1].split(":")[1])
                right_ticks = int(parts[2].split(":")[1])
                dl = (left_ticks - self.prev_left) * (2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV)
                dr = (right_ticks - self.prev_right) * (2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV)
                self.prev_left, self.prev_right = left_ticks, right_ticks
                dc = (dl + dr) / 2
                dtheta = (dr - dl) / BASE_WIDTH
                self.x += dc * math.cos(self.theta + dtheta/2)
                self.y += dc * math.sin(self.theta + dtheta/2)
                self.theta += dtheta

                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = "odom"
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                # Orientation (convert theta to quaternion later if needed)
                self.odom_pub.publish(odom)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

