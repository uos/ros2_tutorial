cmake_minimum_required(VERSION 3.8)
project(ex04_nav2_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

#######
# Nav2 Action Client
######
add_executable(nav2_client
  src/nav2_client.cpp)

ament_target_dependencies(nav2_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs)

install(TARGETS
  nav2_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()