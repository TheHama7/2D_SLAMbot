
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
    libfastcdr-dev\
    rsync\
    build-essential \
    cmake \
    git \
    wget \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*\
&& ln -s /usr/bin/python3 /usr/bin/python \
&& pip3 install pyserial
ENV ROS_DISTRO=humble

RUN ln -sf /usr/bin/python3 /usr/bin/python

# Source ROS 2 installation
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Copy ESP32 app to a temporary holding location
RUN mkdir -p esp32/
WORKDIR /esp32
COPY esp32/ ./esp32/
# Set up workspace
WORKDIR /ros2_ws

# Copy your project files
COPY src/ ./src/


# Auto-source workspace for container sessions
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc



CMD ["bash"]
