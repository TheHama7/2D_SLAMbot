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
    libfastcdr-dev\
    rsync\
    build-essential \
    cmake \
    git \
    wget \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*\
&& ln -s /usr/bin/python3 /usr/bin/python

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

RUN ln -sf /usr/bin/python3 /usr/bin/python

# Create and setup the micro-ROS workspace
RUN mkdir -p /microros_ws/src
WORKDIR /microros_ws
# Source ROS 2 installation
SHELL ["/bin/bash", "-c"]
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

# ---- Pre-create firmware workspace & install ESP-IDF toolchain ----
# Doing this at build-time avoids heavy first-run work when you start a container.
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /microros_ws/install/local_setup.bash && \
    ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32 && \
    ros2 run micro_ros_setup configure_firmware.sh esp32 --transport serial

# Add ESP-IDF toolchain to PATH so CMake finds the compilers at runtime, too
ENV IDF_PATH=/firmware/toolchain/esp-idf
ENV PATH="${PATH}:/firmware/toolchain/espressif/tools/xtensa-esp32-elf/esp-2020r2-8.2.0/xtensa-esp32-elf/bin"

# Help CMake find Fast CDR / Fast DDS when libmicroros builds ROS interfaces
# (colcon runs in a minimal env during the firmware build)
RUN mkdir -p /firmware/toolchain/fastrtps && \
    ln -sf /opt/ros/$ROS_DISTRO/lib/cmake/fastcdr  /firmware/toolchain/fastrtps/fastcdr  || true && \
    ln -sf /opt/ros/$ROS_DISTRO/lib/cmake/fastdds  /firmware/toolchain/fastrtps/fastdds  || true
ENV CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:/firmware/toolchain/fastrtps:${CMAKE_PREFIX_PATH}"

# Copy ESP32 app to a temporary holding location
RUN mkdir -p esp32/
WORKDIR /esp32
COPY esp32/ ./esp32/
COPY scripts/build_and_flash.sh /usr/local/bin/build_and_flash.sh
# Set up workspace
WORKDIR /ros2_ws

# Copy your project files
COPY src/ ./src/

WORKDIR /
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /microros_ws/install/setup.bash' >> ~/.bashrc

ENV PATH="${PATH}:/firmware/toolchain/espressif/tools/xtensa-esp32-elf/esp-2020r2-8.2.0/xtensa-esp32-elf/bin"

CMD ["bash"]
