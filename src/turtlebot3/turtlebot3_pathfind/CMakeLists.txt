cmake_minimum_required(VERSION 3.8)  # Increased minimum version
project(turtlebot3_pathfind)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies (MODERN APPROACH)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(turtlebot3_driver
  src/Turtlebot3Driver.cpp
  src/main.cpp
)

# MODERN DEPENDENCY HANDLING
ament_target_dependencies(turtlebot3_driver
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install targets
install(TARGETS turtlebot3_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()