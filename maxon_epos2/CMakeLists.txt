cmake_minimum_required(VERSION 3.5)
project(maxon_epos2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mobile_base_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  mobile_base_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}_core SHARED
  src/epos_communication.cpp
  src/epos_controller.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    include
)
target_link_libraries(${PROJECT_NAME}_core
  EposCmd
)

ament_target_dependencies(${PROJECT_NAME}_core ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(${PROJECT_NAME}_node
  src/epos_controller_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    include
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

install(TARGETS ${PROJECT_NAME}_core
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()