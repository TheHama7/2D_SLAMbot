import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32
import serial
import threading
import re

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.get_logger().info('Connected to Arduino')

        # Publishers
        self.left_pub = self.create_publisher(Int32, 'encoder/left', 10)
        self.right_pub = self.create_publisher(Int32, 'encoder/right', 10)
        self.imu_pub = self.create_publisher(Vector3, 'imu/accel', 10)

        # Subscriber
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Thread for serial reading
        threading.Thread(target=self.serial_read_loop, daemon=True).start()

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        cmd_str = f"CMD {v:.3f} {w:.3f}\n"
        self.serial_port.write(cmd_str.encode('utf-8'))

    def serial_read_loop(self):
        pattern = re.compile(r"ENC (-?\d+) (-?\d+) IMU ([\d\.\-]+) ([\d\.\-]+) ([\d\.\-]+)")
        while rclpy.ok():
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                continue
            match = pattern.match(line)
            if match:
                left, right, ax, ay, az = match.groups()
                self.publish_data(int(left), int(right), float(ax), float(ay), float(az))

    def publish_data(self, left, right, ax, ay, az):
        self.left_pub.publish(Int32(data=left))
        self.right_pub.publish(Int32(data=right))
        self.imu_pub.publish(Vector3(x=ax, y=ay, z=az))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
