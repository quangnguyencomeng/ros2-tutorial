#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AckermannKinematicsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AckermannKinematicsNode() : LifecycleNode("ackermann_kinematics_node")
  {
    declare_parameter("wheelbase", 1.5);
    declare_parameter("track_width_front", 1.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    wheelbase_   = get_parameter("wheelbase").as_double();
    track_width_ = get_parameter("track_width_front").as_double();

    // FIX 1: bỏ dynamic_pointer_cast, create_publisher trả về LifecyclePublisher luôn
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10));
    wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_commands", rclcpp::QoS(10));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "/cmd_ackermann", 10,
      [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        cmd_callback(msg);
      });

    timer_ = create_wall_timer(50ms, [this]() { update(); });
    timer_->cancel();

    path_msg_.header.frame_id = "odom";
    RCLCPP_INFO(get_logger(), "Configured. wheelbase=%.2f, track=%.2f", wheelbase_, track_width_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    // gọi parent để chuyển state đúng cách
    LifecycleNode::on_activate(state);
    odom_pub_->on_activate();
    path_pub_->on_activate();
    wheel_pub_->on_activate();

    path_msg_.poses.clear(); 
    x_ = 0; y_ = 0; theta_ = 0;
    
    timer_ = create_wall_timer(50ms, [this]() { update(); });
    timer_->reset(); 
    RCLCPP_INFO(get_logger(), "Activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_deactivate(state);
    odom_pub_->on_deactivate();
    path_pub_->on_deactivate();
    wheel_pub_->on_deactivate();
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    odom_pub_.reset(); 
    path_pub_.reset();
    cmd_sub_.reset();  
    timer_.reset();
    wheel_pub_.reset();
    tf_broadcaster_.reset();
    RCLCPP_INFO(get_logger(), "Cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

private:
  void cmd_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    current_speed_    = msg->speed;
    current_steering_ = msg->steering_angle;
  }

  void update()
  {
    auto now = get_clock()->now();
    double dt = 0.05;

    double omega = (wheelbase_ > 1e-9)
      ? (current_speed_ / wheelbase_) * std::tan(current_steering_)
      : 0.0;

    x_     += current_speed_ * std::cos(theta_) * dt;
    y_     += current_speed_ * std::sin(theta_) * dt;
    theta_ += omega * dt;

    double steering_left = 0.0;
  double steering_right = 0.0;

  if (std::abs(current_steering_) > 1e-4) {
    double R = wheelbase_ / std::tan(current_steering_);

    steering_left  = std::atan(wheelbase_ / (R - track_width_ / 2.0));
    steering_right = std::atan(wheelbase_ / (R + track_width_ / 2.0));
  } else {
    steering_left = steering_right = current_steering_; // Đi thẳng
  }

    // TF: odom -> base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation.z = std::sin(theta_ / 2.0);
    tf.transform.rotation.w = std::cos(theta_ / 2.0);
    tf_broadcaster_->sendTransform(tf);

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x  = x_;
    odom.pose.pose.position.y  = y_;
    odom.pose.pose.orientation = tf.transform.rotation;
    odom.twist.twist.linear.x  = current_speed_;
    odom.twist.twist.angular.z = omega;
    odom_pub_->publish(odom);

    // Path
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose   = odom.pose.pose;
    path_msg_.header.stamp = now;
    path_msg_.poses.push_back(ps);
    path_pub_->publish(path_msg_);


    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {current_speed_, steering_left, steering_right};
    wheel_pub_->publish(wheel_msg);
  }

  double wheelbase_{1.5}, track_width_{1.0};
  double x_{0}, y_{0}, theta_{0};
  double current_speed_{0}, current_steering_{0};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr     path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr     cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char ** argv)
{
  // FIX 3: LifecycleNode phải dùng Executor, không dùng rclcpp::spin() trực tiếp
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannKinematicsNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}