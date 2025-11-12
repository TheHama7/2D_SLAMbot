#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
import serial
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw):
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

class ESP32Odom(Node):
    def __init__(self):
        super().__init__('esp32_odom')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.033)  # m
        self.declare_parameter('wheel_base', 0.18)     # m

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.05)

        self.imu_pub  = self.create_publisher(Imu, "/imu/data", 50)
        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 50)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.last = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.step)  # 50 Hz

        # Covariances (tune as needed)
        self.imu_lin_acc_cov = [0.05,0,0, 0,0.05,0, 0,0,0.05]
        self.imu_ang_vel_cov = [0.02,0,0, 0,0.02,0, 0,0,0.02]
        self.odom_pose_cov = [1e-3,0,0, 0,1e-3,0, 0,0,1e-2]
        self.odom_twist_cov = [1e-3,0,0, 0,1e-3,0, 0,0,1e-2]

        self.get_logger().info("ESP32 odom bridge started.")

    def step(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception:
            return
        if not line.startswith("DATA"):
            return
        parts = line.split(',')
        if len(parts) != 9:
            return
        _, ax, ay, az, gx, gy, gz, rpmA, rpmB = parts
        ax, ay, az = float(ax), float(ay), float(az)
        gx, gy, gz = float(gx), float(gy), float(gz)
        rpmA, rpmB = float(rpmA), float(rpmB)

        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        if dt <= 0.0: dt = 1e-3
        self.last = now

        r = float(self.get_parameter('wheel_radius').value)
        L = float(self.get_parameter('wheel_base').value)
        v_left  = (rpmA/60.0) * (2*math.pi*r)
        v_right = (rpmB/60.0) * (2*math.pi*r)
        v = 0.5*(v_left + v_right)
        w = (v_right - v_left)/L

        # integrate encoders only (IMU yaw is fused by EKF)
        self.yaw += w*dt
        self.x += v*math.cos(self.yaw)*dt
        self.y += v*math.sin(self.yaw)*dt

        # IMU message
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "base_link"
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.linear_acceleration_covariance = self.imu_lin_acc_cov
        imu.angular_velocity_covariance = self.imu_ang_vel_cov
        self.imu_pub.publish(imu)

        # wheel odom (encoder-only)
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = yaw_to_quat(self.yaw)
        od.pose.covariance = [0.0]*36
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        od.twist.covariance = [0.0]*36
        self.odom_pub.publish(od)

def main():
    rclpy.init()
    node = ESP32Odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
