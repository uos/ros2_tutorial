#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"

#include <ex04_msgs/action/wait_for_rviz_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

namespace ex04_actions
{

class WaitForRvizPoseActionServer : public rclcpp::Node
{
public:
  using WaitForRvizPose = ex04_msgs::action::WaitForRvizPose;
  using GoalHandleWaitForRvizPose = rclcpp_action::ServerGoalHandle<WaitForRvizPose>;
 
  WaitForRvizPoseActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("wait_for_rviz_pose_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<WaitForRvizPose>(
      this,
      "wait_for_rviz_pose",
      std::bind(&WaitForRvizPoseActionServer::on_goal_received, this, _1, _2),
      std::bind(&WaitForRvizPoseActionServer::on_cancel, this, _1),
      std::bind(&WaitForRvizPoseActionServer::on_goal_accepted, this, _1));

    // parameter that defines the rate of cancel request checks inside the execution loop
    tick_rate_ = this->declare_parameter("tick_rate", 100.0);

    std::cout << "Action Server Started." << std::endl;
  }

private:
  rclcpp_action::Server<WaitForRvizPose>::SharedPtr action_server_;

  std::atomic_bool goal_canceled_ = false;
  double tick_rate_ = 100.0;

  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WaitForRvizPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)goal;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandleWaitForRvizPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    goal_canceled_ = true;

    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_goal_accepted(const std::shared_ptr<GoalHandleWaitForRvizPose> goal_handle)
  {
    using namespace std::placeholders;

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WaitForRvizPoseActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaitForRvizPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaitForRvizPose::Feedback>();
    auto result = std::make_shared<WaitForRvizPose::Result>();

    std::chrono::duration<float> d_ticked(0.0);
    std::chrono::duration<float> d_tick(1.0 / tick_rate_);
    std::chrono::duration<float> d_wait_for(goal->duration);

    bool pose_found = false;

    auto subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped& msg){
        std::cout << "I heard something!" << std::endl;
        pose_found = true;
        result->pose = msg;
      }
    );

    bool time_exceeded = false;
    
    // subscriber is running
    while(!pose_found)
    {
      // wait
      this->get_clock()->sleep_for(d_tick);
      d_ticked += d_tick;

      if(goal_canceled_)
      {
        break;
      }

      if(d_wait_for > 0.0s && d_ticked > d_wait_for)
      {
        // time exceeded
        time_exceeded = true;
        break;
      }
    }

    if(goal_canceled_)
    {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled.");
      goal_canceled_ = false;
    } else if(time_exceeded) {
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled. Reason: time exceeded.");
    } else {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }
  }
};  // class WaitForRvizPoseActionServer

}  // namespace ex04_actions

int main(int argc, char** argv)
{
  using namespace ex04_actions;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaitForRvizPoseActionServer>());
  rclcpp::shutdown();
  return 0;
}