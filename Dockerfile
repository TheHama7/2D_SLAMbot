FROM ros:humble-ros-base

RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# -----------------------------
# System & ROS dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-slam-toolbox \
    ros-humble-rplidar-ros \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    build-essential \
    cmake \
    git \
    udev \
 && rm -rf /var/lib/apt/lists/*

# Python deps
RUN pip3 install --no-cache-dir pyserial

# Make python -> python3
RUN ln -sf /usr/bin/python3 /usr/bin/python

# -----------------------------
# ROS environment
# -----------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# -----------------------------
# Workspace
# -----------------------------

WORKDIR /ros2_ws

COPY ros2/ .

# Build workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# Source workspace on startup
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
