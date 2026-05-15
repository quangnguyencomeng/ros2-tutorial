#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PlannerComparator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PlannerComparator() : Node("planner_comparator_node") {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PlannerComparator::odom_callback, this, std::placeholders::_1));
        
        reset_metrics();
    }

    void send_goal(double x, double y) {
        if (!this->client_ptr_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        start_time_ = this->now();
        is_running_ = true;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&PlannerComparator::result_callback, this, std::placeholders::_1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    void reset_metrics() {
        total_distance_ = 0.0;
        first_odom_received_ = false;
        is_running_ = false;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!is_running_) return;

        if (first_odom_received_) {
            double dx = msg->pose.pose.position.x - last_x_;
            double dy = msg->pose.pose.position.y - last_y_;
            total_distance_ += std::sqrt(dx*dx + dy*dy);
        }
        
        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;
        first_odom_received_ = true;
    }

    void result_callback(const GoalHandleNav::WrappedResult & result) {
        is_running_ = false;
        auto end_time = this->now();
        double duration = (end_time - start_time_).seconds();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            RCLCPP_INFO(this->get_logger(), "Time: %.2f s | Distance: %.2f m", duration, total_distance_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal Failed");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    double total_distance_, last_x_, last_y_;
    bool first_odom_received_, is_running_;
    rclcpp::Time start_time_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerComparator>();
    
    node->send_goal(1.6, 0.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}