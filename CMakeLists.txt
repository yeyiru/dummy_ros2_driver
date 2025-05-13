cmake_minimum_required(VERSION 3.8)
project(dummy_controller)
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
)

add_executable(usb_cam_publisher src/usb_cam_publisher.cpp)

ament_target_dependencies(usb_cam_publisher 
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  image_transport
  OpenCV
  cv_bridge
)

# 安装 Python 模块
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  usb_cam_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()