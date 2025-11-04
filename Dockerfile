# syntax=docker/dockerfile:1.4

FROM ros:humble-ros-base

# Install required ROS 2 and system packages
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
    git wget build-essential cmake \
    libasio-dev libtinyxml2-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


 WORKDIR /opt
RUN git clone https://github.com/eProsima/Fast-CDR.git && \
    cd Fast-CDR && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

    RUN git clone https://github.com/eProsima/Fast-DDS.git && \
    cd Fast-DDS && git checkout v3.0.1 && mkdir build && cd build && \
    cmake .. -DTHIRDPARTY=ON && \
    make -j$(nproc) && make install
    # Setup environment for Fast DDS
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Build micro-ROS Agent
WORKDIR /microros_ws/src
RUN git clone https://github.com/micro-ROS/micro-ROS-Agent.git
WORKDIR /microros_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Set up workspace
WORKDIR /ros2_ws

# Copy your project files
COPY src/ ./src/


# Setup ROS environment in bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /microros_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
