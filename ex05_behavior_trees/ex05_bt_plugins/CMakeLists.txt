cmake_minimum_required(VERSION 3.8)
project(ex05_bt_plugins)

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
find_package(behaviortree_cpp_v3 REQUIRED)

# OLD WAY. TODO:
# - Do target_include_directories instead
# - Check if it working with the behavior tree interface
include_directories(
  include
)

add_library(bt_plugin_dummy_nodes SHARED src/dummy_nodes.cpp)

ament_target_dependencies(bt_plugin_dummy_nodes 
  rclcpp
  rclcpp_action
  geometry_msgs
  ex04_msgs
  behaviortree_cpp_v3)
target_compile_definitions(bt_plugin_dummy_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS 
    bt_plugin_dummy_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  bt_plugin_dummy_nodes
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  ex04_msgs
  behaviortree_cpp_v3
)

ament_package()