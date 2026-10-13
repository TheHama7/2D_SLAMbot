cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install URDF directory
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
