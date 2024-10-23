cmake_minimum_required(VERSION 3.5)
project(imu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Add the executable
add_executable(imu src/imu.cc)

# Link against the necessary libraries
ament_target_dependencies(imu
  rclcpp
  sensor_msgs
  Eigen3
  Boost
)

# Install the executable
install(TARGETS imu
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)
# Ament package
ament_package()
