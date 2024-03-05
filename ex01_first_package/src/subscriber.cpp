#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// only to write something like
// 10ms, 100s
using namespace std::chrono_literals;

// this function is called everytime a message appears 
// on the topic 'important_messages' 
void topic_callback(const std_msgs::msg::String & msg)
{
  std::cout << "I heard: " << msg.data << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create new ROS node with name "publisher"
  auto node = std::make_shared<rclcpp::Node>("subscriber");
  auto subscriber = node->create_subscription<std_msgs::msg::String>(
    "important_messages", 10, topic_callback
    );

  rclcpp::spin(node);
  
  return 0;
}