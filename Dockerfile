# syntax=docker/dockerfile:1.4
FROM ros:humble-ros-base

# Install ROS 2 packages and dependencies
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
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-fastrtps \
    build-essential \
    cmake \
    git \
    wget \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble


# Source ROS 2 installation
SHELL ["/bin/bash", "-c"]
# Create and setup the micro-ROS workspace
RUN mkdir -p /microros_ws/src
WORKDIR /microros_ws/apps

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
# Clone the micro-ROS setup tools
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools
RUN source /opt/ros/$ROS_DISTRO/setup.sh && colcon build

# Source the installed micro-ROS tools
RUN echo "source /microros_ws/install/local_setup.bash" >> ~/.bashrc

# copy esp32 app
COPY esp32/ /microros_ws/apps/esp32/

# Set up workspace
WORKDIR /ros2_ws

# Copy your project files
COPY src/ ./src/

WORKDIR /
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /microros_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
