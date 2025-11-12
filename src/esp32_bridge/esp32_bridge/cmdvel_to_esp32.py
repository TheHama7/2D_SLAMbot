#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp32')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.033)  # m
        self.declare_parameter('wheel_base', 0.18)     # m
        self.declare_parameter('max_rpm', 250.0)       # saturate
        self.declare_parameter('timeout', 0.5)         # seconds

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.05)

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.last_cmd = time.monotonic()
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info("CmdVel→ESP32 started.")

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        r = float(self.get_parameter('wheel_radius').value)
        L = float(self.get_parameter('wheel_base').value)
        max_rpm = float(self.get_parameter('max_rpm').value)

        vL = v - w*L/2.0
        vR = v + w*L/2.0

        rpmA = (vL / (2*math.pi*r)) * 60.0
        rpmB = (vR / (2*math.pi*r)) * 60.0

        rpmA = max(-max_rpm, min(max_rpm, rpmA))
        rpmB = max(-max_rpm, min(max_rpm, rpmB))

        cmd = f"MOTOR,{rpmA:.1f},{rpmB:.1f}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

        self.last_cmd = time.monotonic()

    def watchdog(self):
        if time.monotonic() - self.last_cmd > float(self.get_parameter('timeout').value):
            try:
                self.ser.write(b"MOTOR,0,0\n")
            except Exception:
                pass

def main():
    rclpy.init()
    node = CmdVelToESP32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
