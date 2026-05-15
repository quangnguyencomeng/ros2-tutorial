#ifndef MY_PURE_PURSUIT__PURE_PURSUIT_CONTROLLER_HPP_
#define MY_PURE_PURSUIT__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace my_pure_pursuit
{
class PurePursuitController : public nav2_core::Controller
{
public:
  // Khởi tạo các tham số, log, TF buffer
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  // Hàm quan trọng nhất: Tính vận tốc mỗi chu kỳ
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // Nhận đường đi từ Global Planner
  void setPlan(const nav_msgs::msg::Path & path) override;
  
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  
  rclcpp::Clock::SharedPtr m_clock;
  std::shared_ptr<tf2_ros::Buffer> m_tf;
  nav_msgs::msg::Path m_current_plan;
  double m_lookahead_dist{0.5};
  double m_linear_vel{0.15};
  rclcpp::Logger m_logger{rclcpp::get_logger("PurePursuitController")};
};
}
#endif