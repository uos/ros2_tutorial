cmake_minimum_required(VERSION 3.8)
project(ex04_actions)

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
find_package(ex04_msgs REQUIRED)

#######
# Action Server
######
add_executable(wait_for_rviz_pose_action_server
  src/wait_for_rviz_pose_action_server.cpp)

ament_target_dependencies(wait_for_rviz_pose_action_server
  ex04_msgs
  rclcpp
  rclcpp_action)

add_executable(wait_for_x_action_server
  src/wait_for_x_action_server.cpp)

ament_target_dependencies(wait_for_x_action_server
  ex04_msgs
  rclcpp
  rclcpp_action)

install(TARGETS
  wait_for_rviz_pose_action_server wait_for_x_action_server
  DESTINATION lib/${PROJECT_NAME})


add_executable(wait_for_x_client
  src/wait_for_x_client.cpp)

ament_target_dependencies(wait_for_x_client
  ex04_msgs
  rclcpp
  rclcpp_action)


add_executable(wait_for_x_client_sync
  src/wait_for_x_client_sync.cpp)

ament_target_dependencies(wait_for_x_client_sync
  ex04_msgs
  rclcpp
  rclcpp_action)

# install all nodes
install(TARGETS
  wait_for_rviz_pose_action_server 
  wait_for_x_action_server 
  wait_for_x_client
  wait_for_x_client_sync
  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(
  ex04_msgs
)

ament_package()