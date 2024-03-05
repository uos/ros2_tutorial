#ifndef EX01_FIRST_PACKAGE_MY_PUBLISHER_HPP
#define EX01_FIRST_PACKAGE_MY_PUBLISHER_HPP
// ^- include guards
// -> someone can include this header twice without breaking everything

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// namespace: 
// to make sure we have no name conflicts.
// maybe someone else want to name his class MyPublisher too.
// if its in another namespace, no problem
namespace ex01_first_package
{

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

} // namespace ex01_first_package

#endif // EX01_FIRST_PACKAGE_MY_PUBLISHER_HPP