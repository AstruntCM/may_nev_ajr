cmake_minimum_required(VERSION 3.8)
project(may_nev_ajr)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(trafficLight src/trafficLight.cpp)
ament_target_dependencies(trafficLight rclcpp std_msgs)

add_executable(driverNode src/driver.cpp)
ament_target_dependencies(driverNode rclcpp std_msgs geometry_msgs turtlesim)

install(TARGETS
  trafficLight
  driverNode
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
# simple_sub_node
DESTINATION lib/${PROJECT_NAME})

ament_package()