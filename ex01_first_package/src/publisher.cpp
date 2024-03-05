#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// only to write something like
// 10ms, 100s
using namespace std::chrono_literals;


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create new ROS node with name "publisher"
  auto node = std::make_shared<rclcpp::Node>("publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("important_messages", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  size_t count = 0;
  while(rclcpp::ok())
  {
    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
    
    node->get_clock()->sleep_for(500ms);
    executor.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}