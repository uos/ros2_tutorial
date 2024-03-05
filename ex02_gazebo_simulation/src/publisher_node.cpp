// Include header
#include "ex02_gazebo_simulation/my_publisher.hpp"
// only include header. source code is not known

#include "rclcpp/rclcpp.hpp"

using namespace ex02_gazebo_simulation;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();

  return 0;
}