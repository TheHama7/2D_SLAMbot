import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# --- NEW: Import the xacro library ---
import xacro

# Function to get the path to a package's share directory
def get_share_file(package_name, file_name):
    # This helper is good for robust path construction
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    
    # --- 1. Get Directory Paths ---
    robot_desc_pkg_share_dir = get_package_share_directory('my_robot_description')
    
    # Path to the URDF (robot.urdf.xacro)
    urdf_path = os.path.join(robot_desc_pkg_share_dir, 'urdf', 'robot.urdf.xacro')

    # --- FIX 1: Process XACRO into standard URDF XML ---
    # The Robot State Publisher cannot parse XACRO math (${...}) or macros.
    # We must use the xacro library to convert it first.
    try:
        robot_description_content = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        # Handle case where xacro fails to process (e.g., missing variable definition)
        print(f"Error processing XACRO file {urdf_path}: {e}")
        robot_description_content = "" # Set to empty to allow the error to surface
    
    robot_description = {'robot_description': robot_description_content}

    # Path to parameter files (FIX 2: Correct paths for EKF and SLAM)
    ekf_config_path = os.path.join(
        robot_desc_pkg_share_dir, 
        'config', 
        'ekf.yaml'
    )
    slam_config_path = os.path.join(
        robot_desc_pkg_share_dir, 
        'config', 
        'slam_toolbox_config.yaml'
    )
    return LaunchDescription([
        
        # =================================================================
        # 1. SERIAL COMMUNICATION (ESP32 Bridge Nodes)
        # =================================================================
        # NOTE: Your Python nodes (Issue 2) are failing due to sourcing/metadata issues. 
        # They are kept here, but require re-sourcing or rebuilding the esp32_bridge package.

        # --- CmdVel Sender ---
        Node(
            package='esp32_bridge',
            executable='cmdvel_to_esp32', 
            name='cmdvel_sender', 
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 115200}]
        ),
        
        # --- Odom Receiver ---
        Node(
            package='esp32_bridge',
            executable='esp32_odom_node', 
            name='odom_receiver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 115200}]
        ),

        # =================================================================
        # 2. SENSORS AND TRANSFORMS
        # =================================================================

        # --- RPLIDAR Driver ---
        # NOTE: Using 'rplidar_composition' executable is correct for ROS 2 launch files.
        # Check this port if the operation timeout persists.
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[
                {'serial_port':'/dev/ttyUSB1','baudrate': 115200, 'frame_id':'laser', 'angle_compensate': True}
            ]
        ),
        
        # --- Robot State Publisher (FIXED) ---
        # Sends the static transforms based on the pre-processed URDF.
        # 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description], # <--- Now passes the XML string
            name='robot_state_publisher'
        ),

        # =================================================================
        # 3. LOCALIZATION AND MAPPING
        # =================================================================

        # --- robot_localization EKF (FIXED Config Path) ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path] # <--- Correct path
        ),

        # --- SLAM Toolbox (FIXED Config Path) ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path] # <--- Correct path
        ),

    ])