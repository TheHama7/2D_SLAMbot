import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():
    
    # --- 1. Paths and Variables ---
    robot_desc_pkg_share_dir = get_package_share_directory('my_robot_description')
    
    # URDF Processing
    urdf_path = os.path.join(robot_desc_pkg_share_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_path).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Parameters
    ekf_config_path = os.path.join(robot_desc_pkg_share_dir, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(robot_desc_pkg_share_dir, 'config', 'slam_toolbox_config.yaml')

    # --- 2. Include Official RPLIDAR Launch ---
    # This replaces the manual Node block to avoid port conflicts
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'channel_type': 'serial',
        'serial_port': '/dev/ttyUSB0',
        'serial_baudrate': '115200',
        'frame_id': 'laser',          # MUST match default unless you really need otherwise
        'inverted': 'false',
        'angle_compensate': 'true',
        'respawn':'True',
    'respawn_delay':'2.0',
        }.items()
    )
    rplidar_delayed = TimerAction(
        period=3.0,   # seconds
        actions=[rplidar_launch]
    )

    slam_delayed = TimerAction(
    period=8.0,   
    actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path]
        )
    ]
)
    # --- 3. Return Launch Description ---
    return LaunchDescription([
        
        # RPLIDAR
        rplidar_delayed,
        slam_delayed,
        # ESP32 Bridge: CmdVel Sender
        Node(
            package='esp32_bridge',
            executable='cmdvel_to_esp32', 
            name='cmdvel_sender', 
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 115200}]
        ),
        
        # ESP32 Bridge: Odom Receiver
        Node(
            package='esp32_bridge',
            executable='esp32_odom_node', 
            name='odom_receiver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 115200}]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            name='robot_state_publisher'
        ),

        # EKF Localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

    ])