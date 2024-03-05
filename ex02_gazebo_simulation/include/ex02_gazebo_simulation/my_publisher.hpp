#ifndef EX02_GAZEBO_SIMULATION_MY_PUBLISHER_HPP
#define EX02_GAZEBO_SIMULATION_MY_PUBLISHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ex02_gazebo_simulation
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

} // namespace ex02_gazebo_simulation

#endif // EX02_GAZEBO_SIMULATION_MY_PUBLISHER_HPP