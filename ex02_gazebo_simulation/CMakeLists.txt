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


###########
## ADDITIONAL: EXAMPLE FOR PUBLISHER NODE
## - Header-File, Source-File, Main-Function-File
## Create a library called 'my_publisher'
## Use the library in 'my_publisher_node'

add_library(my_publisher
  src/my_publisher.cpp
)

target_include_directories(my_publisher PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(my_publisher
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS my_publisher
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
)

add_executable(my_publisher_node
  src/publisher_node.cpp
)

target_link_libraries(my_publisher_node
  my_publisher
)

ament_target_dependencies(my_publisher_node
  rclcpp
)

install(TARGETS
  my_publisher_node
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