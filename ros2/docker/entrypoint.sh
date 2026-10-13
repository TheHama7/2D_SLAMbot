#!/usr/bin/env bash
set -euo pipefail

echo "[robot] Starting entrypoint..."

# Optional: show env
echo "[robot] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "[robot] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"

# Wait for hardware (adjust names to your udev symlinks)
echo "[robot] Waiting for ESP32 on /dev/esp32 ..."
for i in {1..60}; do
  if [ -e /dev/esp32 ]; then
    echo "[robot] ESP32 found."
    break
  fi
  sleep 1
done

echo "[robot] Waiting for RPLidar on /dev/rplidar_a1 ..."
for i in {1..60}; do
  if [ -e /dev/rplidar_a1 ]; then
    echo "[robot] RPLidar found."
    break
  fi
  sleep 1
done

# Source ROS + your workspace (THIS is the whole point)
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Quick sanity check (optional, but super useful)
ros2 pkg prefix my_robot_description >/dev/null 2>&1 || {
  echo "[robot] ERROR: my_robot_description not found. Workspace not sourced/built?"
  exit 1
}

echo "[robot] Launching robot..."
exec ros2 launch my_robot_description robot_full.launch.py
