#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-}"

if [[ -z "${PORT}" ]]; then
  echo "❌ Usage: build_and_flash.sh /dev/ttyUSB0"
  exit 1
fi

echo "✅ Sourcing ROS & micro-ROS..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /microros_ws/install/local_setup.bash

# Toolchain & cmake hints (belt-and-suspenders in case the env wasn't inherited)
export IDF_PATH=${IDF_PATH:-/firmware/toolchain/esp-idf}
export PATH="$PATH:/firmware/toolchain/espressif/tools/xtensa-esp32-elf/esp-2020r2-8.2.0/xtensa-esp32-elf/bin"
export CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO:-humble}:/firmware/toolchain/fastrtps:${CMAKE_PREFIX_PATH:-}"

# Sanity checks
echo "🔎 Tool sanity:"
which python || true
which xtensa-esp32-elf-gcc || true

FW_EXT_DIR="/firmware/freertos_apps/microros_esp32_extensions"
FW_APPS_DIR="/firmware/freertos_apps/apps/esp32"
BUILD_DIR="$FW_EXT_DIR/build"

# 1) Create firmware WS if missing (first container run only)
if [[ ! -d "/firmware/freertos_apps" ]]; then
  echo "🧰 Creating firmware workspace (first run)..."
  ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
else
  echo "🧰 Firmware workspace present — skipping create."
fi

# 2) Sync your app (fast incremental copy)
echo "📦 Syncing ESP32 user application to firmware tree..."
rsync -a --delete /esp32/esp32/ "$FW_APPS_DIR/"

# 3) Configure (idempotent)
if [[ ! -f "$FW_EXT_DIR/sdkconfig" ]]; then
  echo "⚙️ First-time firmware configure..."
  ros2 run micro_ros_setup configure_firmware.sh esp32 --transport serial
else
  echo "⚙️ Firmware already configured — skipping."
fi

# 4) Build (use existing CMake tree if present to keep it fast)
echo "🧱 Building firmware..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [[ ! -f Makefile ]]; then
  echo "🔧 No Makefile found — running CMake..."
  cmake ..
fi

make -j"$(nproc)"
echo "✅ Build complete."

# 5) Flash
echo "🔌 Flashing to ${PORT} ..."
make flash ESPPORT="${PORT}"

echo "🎉 Done! micro-ROS firmware is on your ESP32."
echo "👉 Start agent on host:  ros2 run micro_ros_agent micro_ros_agent serial --dev ${PORT}"
