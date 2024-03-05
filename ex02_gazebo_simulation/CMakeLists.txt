cmake_minimum_required(VERSION 3.8)
project(ex02_gazebo_simulation)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(scan_to_pcl 
  src/scan_to_pcl.cpp)
ament_target_dependencies(scan_to_pcl 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
    scan_to_pcl
  DESTINATION lib/${PROJECT_NAME})

########
## Transform Pointcloud using Eigen
add_executable(transform_pcl
  src/transform_pcl.cpp
)

target_link_libraries(transform_pcl
  Eigen3::Eigen
)

ament_target_dependencies(transform_pcl
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  transform_pcl
  DESTINATION lib/${PROJECT_NAME})

ament_package()