cmake_minimum_required(VERSION 3.5)
project(debug_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package
ament_python_install_package(${PROJECT_NAME})

# Register the Python entry point
install(
  PROGRAMS
    debug_node/debug_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()