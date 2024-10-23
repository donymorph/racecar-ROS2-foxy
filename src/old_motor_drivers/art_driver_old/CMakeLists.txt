cmake_minimum_required(VERSION 3.5)
project(art_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")
# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add the library (art_driver_lib) and link it
add_library(art_driver_lib STATIC IMPORTED)
set_target_properties(art_driver_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/libart_driver.a
)

# Add executable
add_executable(motor_driver_node src/art_racecar.cpp)

# Link libraries
target_link_libraries(motor_driver_node
  art_driver_lib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

# Install the executable
install(TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
