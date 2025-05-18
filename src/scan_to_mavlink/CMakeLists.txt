cmake_minimum_required(VERSION 3.5)
project(scan_to_mavlink)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message from msg/ObstacleDistance.msg
rosidl_generate_interfaces(
  "msg/ObstacleDistance.msg"
  DEPENDENCIES std_msgs
)

# Export message support for Python
ament_export_dependencies(rosidl_default_runtime)

# Install Python node directory
install(
  DIRECTORY src/
  DESTINATION lib/
)

# Install node script
install(
  PROGRAMS
  src//scan_to_mavlink_node.py
  DESTINATION lib/
)

ament_package()
