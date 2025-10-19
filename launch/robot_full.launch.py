from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # micro-ROS agent (Arduino)
        ExecuteProcess(
            cmd=['micro-ros-agent', 'serial', '--dev', '/dev/ttyACM0', 'baudrate=115200'],
            name='micro_ros_agent', output='screen'),

        # RPLIDAR
        Node(
            package='rplidar_ros2',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{'serial_port':'/dev/ttyUSB0','frame_id':'laser'}]),

        # robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf_params.yaml']),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['config/slam_toolbox_params.yaml']),

        # rosbridge (WebSocket interface)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port':9090}]),

        # lightweight web server to host dashboard
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080', '--directory', 'web'],
            name='web_server', output='screen'),
    ])
