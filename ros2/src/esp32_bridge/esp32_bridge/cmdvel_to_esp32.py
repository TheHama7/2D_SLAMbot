#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp32')
        
        # --- Kinematic Parameters (Updated to match ESP32 C++ values) ---
        # The ESP32 code used WHEEL_DIAMETER_MM = 85.0
        # Radius = 85.0 mm / 2 = 42.5 mm = 0.0425 m
        self.declare_parameter('wheel_radius', 0.0425) # m (85 mm diameter / 2)
        
        # The ESP32 code used TRACK_WIDTH_MM = 150.0
        # Wheel Base = 150.0 mm = 0.15 m
        self.declare_parameter('wheel_base', 0.15)     # m (150 mm)
        
        # --- Communication and Safety Parameters ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_rpm', 250.0)       # Max saturation value for RPM
        self.declare_parameter('timeout', 0.5)         # seconds (Watchdog time)

        # Retrieve parameters and initialize serial communication
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Successfully opened serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            # Exit or set a flag to prevent further serial writes
            self.ser = None 
            return

        # Setup ROS 2 Subscription and Watchdog Timer
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.last_cmd = time.monotonic()
        self.timer = self.create_timer(0.5, self.watchdog)

        self.get_logger().info("CmdVel→ESP32 bridge started. Listening on /cmd_vel.")

    def cb(self, msg: Twist):
        # Check if serial connection is active
        if self.ser is None:
            return

        # Linear and angular velocity from Twist message
        v = msg.linear.x
        w = msg.angular.z
        
        # Retrieve kinematic parameters
        r = float(self.get_parameter('wheel_radius').value)
        L = float(self.get_parameter('wheel_base').value)
        max_rpm = float(self.get_parameter('max_rpm').value)

        # Inverse Kinematics: Calculate required linear velocity for each wheel
        # vL = v - w*L/2  and  vR = v + w*L/2
        vL = v - w * L / 2.0
        vR = v + w * L / 2.0
        

        # Convert linear velocity (m/s) to RPM
        # RPM = (Linear Velocity / Circumference) * 60
        # Circumference = 2 * pi * r
        circumference = 2 * math.pi * r
        
        # RPM A (Left Wheel) and RPM B (Right Wheel)
        rpmA = (vL / circumference) * 60.0
        rpmB = (vR / circumference) * 60.0

        # Saturation: Clamp the calculated RPMs to the maximum allowable RPM
        rpmA = max(-max_rpm, min(max_rpm, rpmA))
        rpmB = max(-max_rpm, min(max_rpm, rpmB))

        # Format command string exactly as expected by the ESP32: "MOTOR,A_RPM,B_RPM\n"
        cmd = f"{rpmA:.1f},{rpmB:.1f}\n"
        
        try:
            self.ser.write(cmd.encode())
            # self.get_logger().info(f"Sent: {cmd.strip()}") # Uncomment for debugging
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

        self.last_cmd = time.monotonic()

    def watchdog(self):
        # Check if serial connection is active and if the timeout has elapsed
        if self.ser is None:
            return
            
        if time.monotonic() - self.last_cmd > float(self.get_parameter('timeout').value):
            self.get_logger().warn("Watchdog timeout: Sending stop command (MOTOR,0,0).")
            try:
                # Send the exact stop command the ESP32 expects
                self.ser.write(b"0,0\n")
            except Exception:
                pass
            # Update last_cmd to prevent continuous warnings if the serial connection is still open but unresponsive
            self.last_cmd = time.monotonic()


def main():
    rclpy.init()
    node = CmdVelToESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown, ensure motors are stopped
        if node.ser is not None:
             node.ser.write(b"MOTOR,0,0\n")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()