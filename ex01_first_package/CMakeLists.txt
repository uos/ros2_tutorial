cmake_minimum_required(VERSION 3.8)
project(ex01_first_package)

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

add_executable(publisher 
  src/publisher.cpp)
ament_target_dependencies(publisher 
  rclcpp
  std_msgs)

add_executable(publisher_class 
  src/publisher_class.cpp)
ament_target_dependencies(publisher_class 
  rclcpp
  std_msgs)


add_executable(subscriber 
  src/subscriber.cpp)
ament_target_dependencies(subscriber 
  rclcpp
  std_msgs)

add_executable(subscriber_lambda 
  src/subscriber_lambda.cpp)
ament_target_dependencies(subscriber_lambda 
  rclcpp
  std_msgs)

add_executable(subscriber_class 
  src/subscriber_class.cpp)
ament_target_dependencies(subscriber_class 
  rclcpp
  std_msgs)
  
install(TARGETS
    publisher
    publisher_class
    subscriber
    subscriber_lambda
    subscriber_class
  DESTINATION lib/${PROJECT_NAME})



###########
## ADDITIONAL: EXAMPLE FOR PUBLISHER NODE
## - Header-File, Source-File, Main-Function-File
## Create a library called 'my_publisher'
## Use the library in 'publisher_lib'

## 1. LIBRARY
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
)

install(TARGETS my_publisher
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
)

## 2. Node that uses library
## Node knows only the header file and the pre-compiled 
## library code of my_publisher
add_executable(publisher_lib
  src/publisher_lib.cpp
)

target_link_libraries(publisher_lib
  my_publisher
)

ament_target_dependencies(publisher_lib
  rclcpp
)

install(TARGETS
  publisher_lib
  DESTINATION lib/${PROJECT_NAME})

ament_package()