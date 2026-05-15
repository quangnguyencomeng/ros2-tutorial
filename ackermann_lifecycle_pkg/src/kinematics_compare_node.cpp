/**
 * Bài 2: Ackermann vs DiffDrive — ROS2 Node, visualize realtime trong RViz2
 *
 * Publish:
 *   /path/ackermann   nav_msgs/msg/Path  (đỏ)
 *   /path/diffdrive   nav_msgs/msg/Path  (xanh)
 *   /odom/ackermann   nav_msgs/msg/Odometry
 *   /odom/diffdrive   nav_msgs/msg/Odometry
 *
 * Chạy:
 *   ros2 run ackermann_lifecycle_pkg kinematics_compare_node
 */

#include <chrono>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

// ─────────────────────────────────────────────
// State structs
// ─────────────────────────────────────────────
struct State { double x{0}, y{0}, theta{0}; };

// ─────────────────────────────────────────────
// Kinematics
// ─────────────────────────────────────────────
State updateAckermann(State s, double v, double phi, double L, double dt)
{
    double omega = (std::abs(L) > 1e-9) ? (v / L) * std::tan(phi) : 0.0;
    s.x     += v * std::cos(s.theta) * dt;
    s.y     += v * std::sin(s.theta) * dt;
    s.theta += omega * dt;
    return s;
}

State updateDiffDrive(State s, double v, double phi, double L, double dt)
{
    // Convert Ackermann phi → omega để so sánh công bằng cùng input
    double omega = (std::abs(L) > 1e-9) ? (v / L) * std::tan(phi) : 0.0;
    s.x     += v * std::cos(s.theta) * dt;
    s.y     += v * std::sin(s.theta) * dt;
    s.theta += omega * dt;
    return s;
}

// ─────────────────────────────────────────────
// Command scenario (giống bài CSV)
// ─────────────────────────────────────────────
struct Cmd { double v, phi; };

Cmd getCommand(double t)
{
    if      (t < 2.0) return {1.5,  0.0};
    else if (t < 5.0) return {1.5,  0.3};   // rẽ trái
    else if (t < 7.0) return {1.5,  0.0};
    else              return {1.5, -0.3};    // rẽ phải
}

// ─────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────
class KinematicsCompareNode : public rclcpp::Node
{
public:
    KinematicsCompareNode() : Node("kinematics_compare_node")
    {
        // Publishers
        ack_path_pub_  = create_publisher<nav_msgs::msg::Path>("/path/ackermann", 10);
        diff_path_pub_ = create_publisher<nav_msgs::msg::Path>("/path/diffdrive",  10);
        ack_odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("/odom/ackermann", 10);
        diff_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/diffdrive",  10);

        // Path headers
        ack_path_.header.frame_id  = "odom";
        diff_path_.header.frame_id = "odom";

        // CSV export
        csv_.open("/home/ngxwang/ros2_ws/src/ackermann_lifecycle_pkg/trajectory.csv");
        csv_ << "time,x_diff,y_diff,theta_diff,"
                "x_ackermann,y_ackermann,theta_ackermann\n";

        // Timer 50ms = 20Hz, realtime
        timer_ = create_wall_timer(50ms, [this]() { update(); });

        RCLCPP_INFO(get_logger(), "Bắt đầu simulate. Mở RViz2 và add 2 topic Path.");
        RCLCPP_INFO(get_logger(), "  /path/ackermann  (đặt color đỏ)");
        RCLCPP_INFO(get_logger(), "  /path/diffdrive  (đặt color xanh)");
    }

    ~KinematicsCompareNode()
    {
        csv_.close();
        RCLCPP_INFO(get_logger(), "Đã export trajectory.csv");
    }

private:
    void update()
    {
        if (sim_time_ > total_time_) {
            RCLCPP_INFO_ONCE(get_logger(), "Simulate xong 10s. CSV đã lưu.");
            timer_->cancel();
            return;
        }

        Cmd cmd = getCommand(sim_time_);

        ack_state_  = updateAckermann(ack_state_,  cmd.v, cmd.phi, wheelbase_, dt_);
        diff_state_ = updateDiffDrive(diff_state_, cmd.v, cmd.phi, wheelbase_, dt_);

        auto now = get_clock()->now();

        // ── Path ──
        appendPath(ack_path_,  ack_state_,  now);
        appendPath(diff_path_, diff_state_, now);
        ack_path_pub_->publish(ack_path_);
        diff_path_pub_->publish(diff_path_);

        // ── Odometry ──
        ack_odom_pub_->publish(makeOdom(ack_state_,  cmd.v, cmd.phi, now, "ackermann_base"));
        diff_odom_pub_->publish(makeOdom(diff_state_, cmd.v, cmd.phi, now, "diffdrive_base"));

        // ── CSV ──
        csv_ << sim_time_          << ","
             << diff_state_.x      << "," << diff_state_.y << "," << diff_state_.theta << ","
             << ack_state_.x       << "," << ack_state_.y  << "," << ack_state_.theta  << "\n";

        sim_time_ += dt_;
    }

    // ── Helper: thêm pose vào path ──
    void appendPath(nav_msgs::msg::Path & path, const State & s,
                    const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = stamp;
        ps.header.frame_id = "odom";
        ps.pose.position.x = s.x;
        ps.pose.position.y = s.y;
        ps.pose.orientation.z = std::sin(s.theta / 2.0);
        ps.pose.orientation.w = std::cos(s.theta / 2.0);
        path.header.stamp = stamp;
        path.poses.push_back(ps);
    }

    // ── Helper: tạo Odometry msg ──
    nav_msgs::msg::Odometry makeOdom(const State & s, double v, double phi,
                                     const rclcpp::Time & stamp,
                                     const std::string & child_frame)
    {
        double omega = (wheelbase_ > 1e-9) ? (v / wheelbase_) * std::tan(phi) : 0.0;
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = child_frame;
        odom.pose.pose.position.x  = s.x;
        odom.pose.pose.position.y  = s.y;
        odom.pose.pose.orientation.z = std::sin(s.theta / 2.0);
        odom.pose.pose.orientation.w = std::cos(s.theta / 2.0);
        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = omega;
        return odom;
    }

    // ── Params ──
    const double wheelbase_   = 1.5;
    const double dt_          = 0.05;
    const double total_time_  = 10.0;
    double sim_time_          = 0.0;

    State ack_state_, diff_state_;

    nav_msgs::msg::Path ack_path_, diff_path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     ack_path_pub_, diff_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ack_odom_pub_, diff_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream csv_;
};

// ─────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsCompareNode>());
    rclcpp::shutdown();
    return 0;
}