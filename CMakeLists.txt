cmake_minimum_required(VERSION 3.8)
project(dummy)
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
)

add_executable(dummy_controller src/dummy_controller.cpp)

ament_target_dependencies(dummy_controller 
  rclcpp 
  serial 
  geometry_msgs
  std_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  dummy_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
