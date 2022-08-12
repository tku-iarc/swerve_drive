cmake_minimum_required(VERSION 3.5)
project(vehicle_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mobile_base_msgs REQUIRED)
find_package(wheel_controller REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  tf2_geometry_msgs
  rclcpp_lifecycle
  mobile_base_msgs
  wheel_controller
  sensor_msgs
  std_msgs
  nav_msgs
  tf2_ros
  rclcpp
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/vehicle_controller.cpp
  src/kinematics.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/vehicle_controller_node.cpp
  src/vehicle_controller.cpp
  src/kinematics.cpp
)

target_include_directories(
  ${PROJECT_NAME}_node
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
	DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()