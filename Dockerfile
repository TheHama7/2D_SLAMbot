FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    git \
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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* 

WORKDIR /ros2_ws
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && colcon build

ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY config/ ./config/
COPY launch/ ./launch/


ENTRYPOINT source /opt/ros/$ROS_DISTRO/setup.bash && \
           source install/setup.bash && \
           ros2 launch robot_bringup robot_full.launch.py
