#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create new ROS node with name "publisher"
  auto node = std::make_shared<rclcpp::Node>("subscriber");
  auto subscriber = node->create_subscription<std_msgs::msg::String>(
    "important_messages", 10,
    [&](const std_msgs::msg::String& msg){
      std::cout << "I heard: " << msg.data << std::endl;
      }
    );

  rclcpp::spin(node);
  
  return 0;
}