#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>


#include "ex04_msgs/action/wait_for_x.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class WaitForXClient : public rclcpp::Node
{
public:
  using WaitForX = ex04_msgs::action::WaitForX;
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaitForX>;

  WaitForXClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("wait_for_x_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<WaitForX>(
      this,
      "wait_for_x");
  }

  GoalHandle::SharedPtr send_goal(float duration)
  {
    return send_goal_async(duration).get();
  }

  std::shared_future<GoalHandle::SharedPtr> send_goal_async(float duration)
  {
    using namespace std::placeholders;

    std::cout << "Waiting connection to action server..." << std::endl;
    if (!this->client_ptr_->wait_for_action_server()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = WaitForX::Goal();
    goal_msg.duration = duration;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<WaitForX>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&WaitForXClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&WaitForXClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&WaitForXClient::result_callback, this, _1);
    
    finished_ = false;
    return this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void spin_until_finished()
  {
    while(!finished_ && rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());
      this->get_clock()->sleep_for(10ms);
    }
  }

private:
  rclcpp_action::Client<WaitForX>::SharedPtr client_ptr_;
  std::atomic_bool finished_ = false;

  void goal_response_callback(const GoalHandle::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const WaitForX::Feedback> feedback)
  {
    // rclcpp::Duration estimated_time_remaining = feedback->estimated_time_remaining;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Estimated time remaining: " << estimated_time_remaining.seconds() << "s");
  }

  void result_callback(const GoalHandle::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::cout << "Node finished." << std::endl;
    finished_ = true;
  }
};  // class WaitForXClient

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto wait_for_x_client = std::make_shared<WaitForXClient>();

  auto fut = wait_for_x_client->send_goal_async(10.0);
  std::cout << "Waiting for 10 seconds..." << std::endl;
  wait_for_x_client->spin_until_finished();

  // This would be better if it worked:
  // if(rclcpp::spin_until_future_complete(wait_for_x_client, fut, 10s)
  //   != rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   std::cout << "ERROR!" << std::endl;
  // }
  std::cout << "Done." << std::endl;

  rclcpp::shutdown();
  return 0;
}