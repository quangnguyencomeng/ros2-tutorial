#include "my_pure_pursuit/pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_pure_pursuit {

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) 
{
  m_tf = tf;
  auto node = parent.lock();

  m_clock = node->get_clock();
  
  // Khai báo tham số để có thể chỉnh trong file YAML
  nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_dist", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".lookahead_dist", m_lookahead_dist);
  
  RCLCPP_INFO(m_logger, "PurePursuitController configured!");
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path) {
  m_current_plan = path;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/, 
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/) 
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.header.stamp = m_clock->now();

  if (m_current_plan.poses.empty()) return cmd;

  std::string path_frame = m_current_plan.header.frame_id;
  if (path_frame.empty()) path_frame = "map"; 

  geometry_msgs::msg::TransformStamped transform;
  transform = m_tf->lookupTransform("base_link", path_frame, tf2::TimePointZero);
  

  // --- PHẦN CODE MỚI: TÍNH KHOẢNG CÁCH ĐẾN ĐÍCH CUỐI CÙNG ---
  geometry_msgs::msg::PoseStamped final_pose = m_current_plan.poses.back();
  geometry_msgs::msg::PoseStamped local_final_pose;
  tf2::doTransform(final_pose, local_final_pose, transform);
  
  double dist_to_goal = std::hypot(local_final_pose.pose.position.x, local_final_pose.pose.position.y);

  // 1. Nếu đã tới rất gần đích (cách < 0.1 mét), dừng xe ngay lập tức!
  if (dist_to_goal < 0.1) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    RCLCPP_INFO(m_logger, "Reached goal.");
    return cmd;
  }

  // 2. Tính toán vận tốc (Giảm tốc khi cách đích nhỏ hơn tầm nhìn lookahead_dist)
  double current_lin_vel = m_linear_vel;
  if (dist_to_goal < m_lookahead_dist) {
    // Ép tốc độ giảm dần theo tỷ lệ khoảng cách
    current_lin_vel = m_linear_vel * (dist_to_goal / m_lookahead_dist);
    
    // Giữ một mức ga tối thiểu (0.05) để xe không bị "chết đứng" trước khi chạm đích
    if (current_lin_vel < 0.05) current_lin_vel = 0.05;
  }
  // ---------------------------------------------------------

  geometry_msgs::msg::PoseStamped local_target;
  bool found_pt = false;

  for (const auto & p : m_current_plan.poses) {
    geometry_msgs::msg::PoseStamped p_with_frame = p;
    p_with_frame.header.frame_id = path_frame;

    geometry_msgs::msg::PoseStamped local_p;
    tf2::doTransform(p_with_frame, local_p, transform);

    double dist = std::hypot(local_p.pose.position.x, local_p.pose.position.y);
    
    if (dist >= m_lookahead_dist && local_p.pose.position.x > 0.0) {
      local_target = local_p;
      found_pt = true;
      break;
    }
  }

  if (!found_pt) {
    local_target = local_final_pose; // Tận dụng luôn biến local_final_pose đã tính ở trên
  }

  double x = local_target.pose.position.x;
  double y = local_target.pose.position.y;
  double L2 = x*x + y*y;
  
  double curvature = (L2 > 0.001) ? (2.0 * y) / L2 : 0.0;

  // 3. Sử dụng current_lin_vel (đã có tính năng giảm tốc) thay vì m_linear_vel cố định
  cmd.twist.linear.x = current_lin_vel;
  cmd.twist.angular.z = current_lin_vel * curvature; 

  // In ra log để dễ debug
  // RCLCPP_INFO(m_logger, "Dist to goal: %.2f | Vel: %.2f | Ang: %.2f", dist_to_goal, cmd.twist.linear.x, cmd.twist.angular.z);

  return cmd;
}


void PurePursuitController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {}

} // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_pure_pursuit::PurePursuitController, nav2_core::Controller)