// Include header
#include "ex01_first_package/my_publisher.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ex01_first_package
{

MyPublisher::MyPublisher()
: rclcpp::Node("publisher")
, count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("important_messages", 10);
  timer_ = this->create_wall_timer(
    500ms, std::bind(&MyPublisher::timer_callback, this));
}

void MyPublisher::timer_callback()
{
  std_msgs::msg::String message;
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

} // namespace ex01_first_package
