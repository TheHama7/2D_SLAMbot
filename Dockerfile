FROM ros:humble-ros-base

# Install ROS 2 packages + pip + dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-slam-toolbox \
    ros-humble-rplidar-ros \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-diagnostics \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rosbridge-server \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick \
    ros-humble-imu-complementary-filter \
    ros-humble-micro-ros-agent \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

 RUN pip3 install -U micro_ros_setup

# Set up workspace
WORKDIR /ros2_ws

# Copy your project files
COPY launch/ ./launch/
COPY webpage/ ./webpage/
COPY config/ ./config/
COPY ArduinoMC/ ./ArduinoMC/

# Setup ROS environment in bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
