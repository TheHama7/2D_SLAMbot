import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = {
        'robot_description': xacro.process_file(urdf_path).toxml()
    }

    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')

    # --- Core robot nodes (NO DELAY) ---
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

   


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen'
    )

    # --- RPLIDAR (delayed) ---
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar_a1',
            'serial_baudrate': '115200',
            'frame_id': 'laser',
        }.items()
    )

    rplidar_delayed = TimerAction(period=3.0, actions=[rplidar])

    # --- SLAM (delayed more) ---
    slam_delayed = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_config],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_pub,
        
        Node(
            package='esp32_bridge',
            executable='cmdvel_to_esp32',
            name='cmdvel_sender',
            output='screen',
            parameters=[{
                'port': '/dev/esp32',
                'baud': 115200
            }]
        ),

        # -------------------------
        # ESP32 → Odometry + IMU
        # -------------------------
        Node(
            package='esp32_bridge',
            executable='esp32_odom_node',
            name='odom_receiver',
            output='screen',
            parameters=[{
                'port': '/dev/esp32',
                'baud': 115200
            }]
        ),

        ekf_node,
        rplidar_delayed,
        slam_delayed,
    ])
