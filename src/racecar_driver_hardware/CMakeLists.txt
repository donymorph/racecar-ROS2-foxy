cmake_minimum_required(VERSION 3.5)
project(racecar_driver_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add the library (racecar_driver_hardware) and link it
add_library(racecar_driver_hardware SHARED
  src/racecar_system_hardware.cpp
  src/serial_communicator.cpp
)
target_include_directories(racecar_driver_hardware PRIVATE include)
ament_target_dependencies(
  racecar_driver_hardware
  rclcpp
  hardware_interface
  controller_manager
  pluginlib
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface racecar_system_hardware.xml)

# Install the library
install(TARGETS racecar_driver_hardware
  DESTINATION lib
)
# Install the plugin description file
install(
  FILES racecar_system_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
