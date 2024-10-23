cmake_minimum_required(VERSION 3.5)
project(racecar_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add executable
add_executable(racecar_driver_node src/art_racecar.cpp)

# Link libraries
ament_target_dependencies(racecar_driver_node rclcpp geometry_msgs )  

# Install the library and executable
install(TARGETS racecar_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
ament_package()
