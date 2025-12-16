import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Function to get the path to a package's share directory
def get_share_file(package_name, file_name):
    # This helper is good for robust path construction
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    # --- 1. Get Directory Paths ---
    # Assuming your custom nodes are in a package named 'esp32_bridge'
    # Assuming your config files and URDF are in 'my_robot_description'
    robot_desc_pkg_share_dir = get_package_share_directory('my_robot_description')
    
    # Path to the URDF (you must place robot.urdf.xacro inside my_robot_description/urdf/)
    urdf_path = os.path.join(robot_desc_pkg_share_dir, 'urdf', 'robot.urdf.xacro')

    # Read the URDF file content for the robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        
        # =================================================================
        # 1. SERIAL COMMUNICATION (ESP32 Bridge Nodes)
        # =================================================================

        # --- CmdVel Sender (Controller: RPi -> ESP32) ---
        # Subscribes to /cmd_vel and sends MOTOR,rpmA,rpmB commands over serial
        Node(
            package='esp32_bridge',
            executable='cmdvel_to_esp32', 
            name='cmdvel_sender', 
            output='screen',
            parameters=[{
                # Note: This is now the direct port for your ESP32
                'port': '/dev/ttyACM0', 
                'baud': 115200
            }]
        ),
        
        # --- Odom Receiver (Data Acquisition: ESP32 -> RPi) ---
        # Reads ODOM serial packets and publishes /imu/data and /wheel_odom
        Node(
            package='esp32_bridge',
            executable='esp32_odom_node', 
            name='odom_receiver',
            output='screen',
            parameters=[{
                # Must use the same port as the sender
                'port': '/dev/ttyACM0', 
                'baud': 115200
            }]
        ),

        # =================================================================
        # 2. SENSORS AND TRANSFORMS
        # =================================================================

        # --- RPLIDAR A1M8 Driver ---
        Node(
            package='rplidar_ros2',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[
                # Ensure /dev/ttyUSB0 is the correct port for the LiDAR
                {'serial_port':'/dev/ttyUSB0', 'frame_id':'laser'}
            ]
        ),
        
        # --- Robot State Publisher (URDF) ---
        # Broadcasts static transforms (base_link -> sensor frames)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            name='robot_state_publisher'
        ),

        # =================================================================
        # 3. LOCALIZATION AND MAPPING
        # =================================================================

        # --- robot_localization EKF (Data Fusion) ---
        # Fuses /wheel_odom and /imu/data into one clean /odometry/filtered topic
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            # Ensures the path to the config file is correct
            parameters=[os.path.join(robot_desc_pkg_share_dir, 'config', 'ekf_params.yaml')]
        ),

        # --- SLAM Toolbox (Mapping) ---
        # Consumes /scan and /odometry/filtered to build a map and provide the map->odom transform
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            # Ensures the path to the config file is correct
            parameters=[os.path.join(robot_desc_pkg_share_dir, 'config', 'slam_toolbox_params.yaml')]
        ),

        # =================================================================
        # 4. REMOTE INTERFACE (Optional)
        # =================================================================

        # --- rosbridge (WebSocket interface) ---
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port':9090}]
        ),

        # --- Lightweight Web Server ---
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080', '--directory', 'web'],
            name='web_server', output='screen'
        ),
    ])