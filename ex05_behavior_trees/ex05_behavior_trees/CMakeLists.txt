cmake_minimum_required(VERSION 3.8)
project(ex05_behavior_trees)

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
find_package(ex05_bt_plugins REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)


add_executable(bt_dummy src/bt_dummy.cpp)
ament_target_dependencies(bt_dummy
  rclcpp
  rclcpp_action
  geometry_msgs
  ex04_msgs
  ex05_bt_plugins
  behaviortree_cpp_v3
)

add_executable(bt_dummy_groot src/bt_dummy_groot.cpp)
ament_target_dependencies(bt_dummy_groot
  rclcpp
  rclcpp_action
  geometry_msgs
  ex04_msgs
  ex05_bt_plugins
  behaviortree_cpp_v3
)

install(TARGETS
    bt_dummy bt_dummy_groot
  DESTINATION lib/${PROJECT_NAME})

ament_package()