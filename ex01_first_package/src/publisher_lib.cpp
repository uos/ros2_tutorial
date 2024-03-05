// Include header
#include "ex01_first_package/my_publisher.hpp"
// only include header. source code is not known

#include "rclcpp/rclcpp.hpp"

using namespace ex01_first_package;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();

  return 0;
}