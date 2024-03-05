#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>


#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace ex04_nav2_client
{

class Nav2ActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2ActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("nav2_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");
  }

  GoalHandleNavigateToPose::SharedPtr send_goal(geometry_msgs::msg::PoseStamped pose)
  {
    return send_goal_async(pose).get();
  }

  std::shared_future<GoalHandleNavigateToPose::SharedPtr> send_goal_async(geometry_msgs::msg::PoseStamped pose)
  {
    using namespace std::placeholders;

    std::cout << "Waiting for connection to action server..." << std::endl;
    if (!this->client_ptr_->wait_for_action_server()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&Nav2ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Nav2ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Nav2ActionClient::result_callback, this, _1);
    
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
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  std::atomic_bool finished_ = false;

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    rclcpp::Duration estimated_time_remaining = feedback->estimated_time_remaining;
    RCLCPP_INFO_STREAM(this->get_logger(), "Estimated time remaining: " << estimated_time_remaining.seconds() << "s");
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult& result)
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
};  // class Nav2ActionClient

}  // namespace ex04_nav2_client

int main(int argc, char** argv)
{
  using namespace ex04_nav2_client;
  rclcpp::init(argc, argv);
  auto nav2_client = std::make_shared<Nav2ActionClient>();


  std::vector<geometry_msgs::msg::PoseStamped> goal_list;

  // first goal
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.frame_id = "map";
  pose1.header.stamp = nav2_client->now();
  pose1.pose.position.x = 15.0;
  pose1.pose.position.y = 19.0;
  pose1.pose.position.z = 0.0;
  pose1.pose.orientation.x = 0.0;
  pose1.pose.orientation.y = 0.0;
  pose1.pose.orientation.z = 0.0;
  pose1.pose.orientation.w = 1.0;
  goal_list.push_back(pose1);

  // second goal
  geometry_msgs::msg::PoseStamped pose2;
  pose2.header.frame_id = "map";
  pose2.header.stamp = nav2_client->now();
  pose2.pose.position.x = 26.0;
  pose2.pose.position.y = 15.4;
  pose2.pose.position.z = 0.0;
  pose2.pose.orientation.x = 0.0;
  pose2.pose.orientation.y = 0.0;
  pose2.pose.orientation.z = 0.0;
  pose2.pose.orientation.w = 1.0;
  goal_list.push_back(pose2);

  while(rclcpp::ok())
  {
    for(size_t i = 0; i<goal_list.size(); i++)
    {
      std::cout << "Goal " << i+1 << " send!" << std::endl;
      auto goal = goal_list[i];
      goal.header.stamp = nav2_client->now(); // update time
      auto fut = nav2_client->send_goal_async(goal);
      nav2_client->spin_until_finished();

      // TODO: This should be better. Why does it not work?
      // if(rclcpp::spin_until_future_complete(nav2_client, fut) 
      //   != rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   std::cout << "ERROR!" << std::endl;
      // }
    }
  }

  rclcpp::shutdown();
  return 0;
}