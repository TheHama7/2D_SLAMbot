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
        
        # --- KINEMATIC PARAMETERS: UPDATED TO MATCH ESP32 (85mm dia, 150mm track) ---
        self.declare_parameter('wheel_radius', 0.0425)  # m (85mm / 2 / 1000)
        self.declare_parameter('wheel_base', 0.15)     # m (150mm / 1000)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Successfully opened serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None
            return

        self.imu_pub  = self.create_publisher(Imu, "/imu/data", 50)
        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 50)

        # Current robot pose (integrated odometry)
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.last = self.get_clock().now()
        
        # ESP32 sends data at 10ms (100Hz), so we read slightly faster
        self.timer = self.create_timer(0.01, self.step) 

        # Covariances (used for EKF, kept as-is)
        self.imu_lin_acc_cov = [0.05,0,0, 0,0.05,0, 0,0,0.05]
        self.imu_ang_vel_cov = [0.02,0,0, 0,0.02,0, 0,0,0.02]
        self.odom_pose_cov = [1e-3,0,0, 0,1e-3,0, 0,0,1e-2]
        self.odom_twist_cov = [1e-3,0,0, 0,1e-3,0, 0,0,1e-2]

        self.get_logger().info("ESP32 odom bridge started.")

    def step(self):
        if self.ser is None:
            return
        
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception:
            return
            
        # --- SERIAL PARSING FIX ---
        # EXPECTING: ODOM,delta_dist_A_mm,delta_dist_B_mm,ax,ay,az,gx,gy,gz,yaw
        if not line.startswith("ODOM"):  # FIX 1: Change "DATA" to "ODOM"
            return
        parts = line.split(',')
        if len(parts) != 10: # FIX 2: Check for 10 fields (ODOM + 9 values)
            # self.get_logger().warn(f"Bad packet size ({len(parts)}) for: {line}") # Debugging
            return
            
        # FIX 3: Change variable names to match C++ packet order
        _, distA_mm, distB_mm, ax, ay, az, gx, gy, gz, imu_yaw = parts
        
        # Convert all to float
        try:
            distA_mm, distB_mm = float(distA_mm), float(distB_mm)
            ax, ay, az = float(ax), float(ay), float(az)
            gx, gy, gz = float(gx), float(gy), float(gz)
            imu_yaw = float(imu_yaw)
        except ValueError:
            self.get_logger().error(f"Failed to convert data to float: {line}")
            return

        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        if dt <= 0.0: dt = 1e-3
        self.last = now

        L = float(self.get_parameter('wheel_base').value)
        
        # Convert distances (mm) to meters (m)
        distA = distA_mm / 1000.0  # Left Wheel distance traveled (m)
        distB = distB_mm / 1000.0  # Right Wheel distance traveled (m)

        # --- ODOMETRY CALCULATION FIX (Forward Kinematics using Distance) ---
        # Calculate linear (v) and angular (w) velocity based on distances and time
        
        # Linear velocity (m/s)
        v_left  = distA / dt 
        v_right = distB / dt
        
        # Robot center velocity
        v = 0.5 * (v_left + v_right)
        
        # Robot angular velocity (rad/s)
        w = (distB - distA) / (L * dt)
        
        # Integrate Odometry using current velocities and time step (Dead Reckoning)
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        # --------------------------------------------------------------------

        # IMU message (using linear acceleration and angular velocity)
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
        od.pose.covariance = self.odom_pose_cov + [0.0]*(36 - 6) # Use defined covariance
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        od.twist.covariance = self.odom_twist_cov + [0.0]*(36 - 6) # Use defined covariance
        self.odom_pub.publish(od)

def main():
    rclpy.init()
    node = ESP32Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()