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

class WaitForXClientSync : public rclcpp::Node
{
public:
  using WaitForX = ex04_msgs::action::WaitForX;
  using ClientT = rclcpp_action::Client<WaitForX>;
  using GoalHandle = ClientT::GoalHandle;
  using WrappedResult = ClientT::WrappedResult;
  
  WaitForXClientSync(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("wait_for_x_client_sync", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<WaitForX>(
      this,
      "wait_for_x");
  }

  /**
   * Completely synchronized action call
  */
  std::optional<WrappedResult> send_goal(float duration)
  {
    auto res_fut = send_goal_async(duration);

    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), res_fut)
      != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "ERROR waiting for action response");
      return {};
    }

    return res_fut.get();
  }

  /**
   * Synchronized goal call, but asynchronous result call
  */
  std::shared_future<WrappedResult> send_goal_async(float duration)
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

    auto send_goal_options = ClientT::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&WaitForXClientSync::feedback_callback, this, _1, _2);
    
    // this one is almost instantly finished
    auto goal_fut = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // this is required to make sure we wait for the result of our goal request
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_fut)
      != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "ERROR waiting for goal request");
    }

    return this->client_ptr_->async_get_result(goal_fut.get());
  }

private:
  ClientT::SharedPtr client_ptr_;
  std::atomic_bool finished_ = false;

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const WaitForX::Feedback> feedback)
  {
    (void)feedback; // this disables warnings for unused variables
    // rclcpp::Duration estimated_time_remaining = feedback->estimated_time_remaining;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Estimated time remaining: " << estimated_time_remaining.seconds() << "s");
  }

};  // class WaitForXClientSync

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Warning: Sometimes there is an issue:
  // - async_send_goal returns goal_future
  //     - goal_future is quickly finished
  // - aysnc_get_result returns goal future object
  //    - ERROR
  // My guess: 
  // - shortly after async_send_goal, the server has finished its work and sends a result
  // - after that, aysnc_get_result is called but the result is gone already

  // create the action client node
  auto wait_for_x_client = std::make_shared<WaitForXClientSync>();
  // synchronous call
  auto res_opt = wait_for_x_client->send_goal(10.0);
  std::cout << "Done." << std::endl;

  if(res_opt)
  {
    auto res = *res_opt;
    // do something dependend on the result
    switch (res.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(wait_for_x_client->get_logger(), "Goal was aborted");
        return 0;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(wait_for_x_client->get_logger(), "Goal was canceled");
        return 0;
      default:
        RCLCPP_ERROR(wait_for_x_client->get_logger(), "Unknown result code");
        return 0;
    }
  } else {
    std::cout << "No result" << std::endl;
    return 0;
  }
  

  std::cout << "GO ON" << std::endl;

  rclcpp::shutdown();
  return 0;
}