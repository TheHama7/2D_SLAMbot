#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def yaw_to_quat(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('max_rpm', 250.0)
        self.declare_parameter('cmd_timeout', 0.5)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Opened ESP32 serial on {port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise SystemExit

        # ---------------- ROS INTERFACES ----------------
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_cb, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 50)
        self.imu_pub  = self.create_publisher(Imu, "/imu/data", 50)

        # ---------------- STATE ----------------
        self.last_cmd_time = time.monotonic()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # ---------------- TIMERS ----------------
        self.read_timer = self.create_timer(0.01, self.read_serial)   # 100 Hz
        self.watchdog   = self.create_timer(0.1, self.cmd_watchdog)

        self.get_logger().info("ESP32 combined bridge running.")

    # ==========================================================
    # CMD_VEL → MOTOR COMMANDS
    # ==========================================================
    def cmd_cb(self, msg: Twist):
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        max_rpm = self.get_parameter('max_rpm').value

        v = msg.linear.x
        w = msg.angular.z

        vL = v - w * L / 2.0
        vR = v + w * L / 2.0

        circ = 2 * math.pi * r
        rpmL = max(-max_rpm, min(max_rpm, (vL / circ) * 60.0))
        rpmR = max(-max_rpm, min(max_rpm, (vR / circ) * 60.0))

        try:
            self.ser.write(f"{rpmL:.1f},{rpmR:.1f}\n".encode())
        except Exception as e:
            self.get_logger().warn(f"Motor write failed: {e}")

        self.last_cmd_time = time.monotonic()

    def cmd_watchdog(self):
        if time.monotonic() - self.last_cmd_time > self.get_parameter('cmd_timeout').value:
            try:
                self.ser.write(b"0,0\n")
            except Exception:
                pass

    # ==========================================================
    # SERIAL READ → ODOM + IMU
    # ==========================================================
    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception:
            return

        if not line.startswith("ODOM"):
            return

        parts = line.split(',')
        if len(parts) != 10:
            return

        _, dA, dB, ax, ay, az, gx, gy, gz, imu_yaw = parts

        try:
            dA = float(dA) / 1000.0
            dB = float(dB) / 1000.0
            ax, ay, az = float(ax), float(ay), float(az)
            gx, gy, gz = float(gx), float(gy), float(gz)
        except ValueError:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0:
            return

        L = self.get_parameter('wheel_base').value

        vL = dA / dt
        vR = dB / dt
        v = 0.5 * (vL + vR)
        w = (dB - dA) / (L * dt)

        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # ---------------- IMU ----------------
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "base_link"
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        self.imu_pub.publish(imu)

        # ---------------- ODOM ----------------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b"0,0\n")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
