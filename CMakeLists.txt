cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(lifecycle_sensor
  src/lifecycle_sensor.cpp
)

# Link dependencies
ament_target_dependencies(lifecycle_sensor
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Install executable
install(TARGETS
  lifecycle_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
