cmake_minimum_required(VERSION 3.5)

# Name of the component (must match folder name)
set(COMPONENT_NAME "esp32")

# Register the micro-ROS app + IMU driver
idf_component_register(
    SRCS
        "app.c"
        "IMU.cpp" 
        "QMI8658.cpp" 
        "AK09918.cpp"
        "i2c_utils.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        freertos
        micro_ros
)

# Allow long calls (Fixes flash memory call range)
target_compile_options(${COMPONENT_LIB} PRIVATE -mlongcalls)
