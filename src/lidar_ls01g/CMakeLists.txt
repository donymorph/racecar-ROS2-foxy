cmake_minimum_required(VERSION 3.5)
project(ls01g)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_executable(ls01g src/main.cpp src/uart_driver.cpp)

ament_target_dependencies(ls01g rclcpp sensor_msgs std_msgs)


install(TARGETS ls01g
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME})
ament_package()
