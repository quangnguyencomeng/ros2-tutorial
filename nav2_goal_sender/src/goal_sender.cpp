#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
using namespace std;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::placeholders;

class GoalSenderNode : public rclcpp::Node
{
public:
  GoalSenderNode() : Node("goal_sender")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void send_goal(double x, double y, double yaw)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
      RCLCPP_ERROR(get_logger(), "Action server not ready!");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = now();

    goal_msg.pose.pose.position.x  = x;
    goal_msg.pose.pose.position.y  = y;

    // Chuyển đổi góc Yaw (radian) sang Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // TODO: Gán GoalResponseCallback, FeedbackCallback, ResultCallback
    send_goal_options.goal_response_callback = bind(&GoalSenderNode::GoalResponseCallback, this, _1);
    send_goal_options.feedback_callback = bind(&GoalSenderNode::FeedbackCallback, this, _1, _2);
    send_goal_options.result_callback = bind(&GoalSenderNode::ResultCallback, this, _1);

    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "Goal sent: (%.2f, %.2f, %.2f)", x, y, yaw);
  }


private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  
  void GoalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    // TODO
    if (!goal_handle){
        RCLCPP_ERROR(this -> get_logger(), "Goal Refused!");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Goal Accepted");
    }
  }

  void FeedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // TODO: In vị trí hiện tại và estimated_time_remaining
    //lệnh ros2 interface show <package_name/type/interface_name>
    double x = feedback -> current_pose.pose.position.x;
    double y = feedback -> current_pose.pose.position.y;
    double z = feedback -> current_pose.pose.position.z;
    // tuỳ thuộc vào dạng mà khai báo (estimated_time_remaining có dạng sec + nanosec)
    double t = feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec / 1e9;

    RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f] \n Estimated time remaining: %.2f", x, y, z, t);
  }

  void ResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    // TODO: Xử lý SUCCEEDED, ABORTED, CANCELED
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
        auto data = result.result->result;
        RCLCPP_INFO(this->get_logger(), "Succeeded!");
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED){
        RCLCPP_ERROR(this->get_logger(), "Aborted!");
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED){
        RCLCPP_WARN(this->get_logger(), "Cancelled");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalSenderNode>();
  node->send_goal(1.5, -0.5, 0.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

