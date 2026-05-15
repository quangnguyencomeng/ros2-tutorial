#include <chrono>
#include <cmath>
#include <random> // Thư viện tạo nhiễu

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

struct RobotState { double x{0}, y{0}, theta{0}; };

class AckermannNoiseNode : public rclcpp::Node
{
public:
    AckermannNoiseNode() : Node("ackermann_noise_node")
    {
        // Khai báo parameter để dễ dàng test các mức nhiễu (0.001, 0.01, 0.05)
        this->declare_parameter("noise_level", 0.001);
        noise_level_ = this->get_parameter("noise_level").as_double();

        truth_path_pub_ = create_publisher<nav_msgs::msg::Path>("/path/ground_truth", 10);
        noisy_path_pub_ = create_publisher<nav_msgs::msg::Path>("/path/noisy_odom", 10);

        truth_path_.header.frame_id = "odom";
        noisy_path_.header.frame_id = "odom";

        // Khởi tạo bộ sinh nhiễu Gaussian
        generator_ = std::default_random_engine(std::random_device{}());
        noise_dist_ = std::normal_distribution<double>(0.0, noise_level_);

        timer_ = create_wall_timer(50ms, [this]() { update(); });

        RCLCPP_INFO(get_logger(), "Starting Noise Simulation with std_dev = %.3f", noise_level_);
    }

private:
    void update()
    {
        if (sim_time_ > 15.0) {
            RCLCPP_INFO_ONCE(get_logger(), "Simulation Completed.");
            timer_->cancel();
            return;
        }

        // 1. Lệnh điều khiển lý tưởng (Ví dụ: chạy vòng tròn)
        double cmd_v = 1.0;
        double cmd_phi = 0.2; 

        // 2. Cập nhật Ground Truth (Không nhiễu)
        truth_state_ = updateKinematics(truth_state_, cmd_v, cmd_phi, dt_);

        // 3. Tạo tín hiệu cảm biến thực tế bị nhiễu (Thêm Gaussian noise)
        double noisy_v = cmd_v + noise_dist_(generator_);
        double noisy_phi = cmd_phi + noise_dist_(generator_);

        // 4. Cập nhật Odometry (Dựa trên tín hiệu nhiễu)
        noisy_state_ = updateKinematics(noisy_state_, noisy_v, noisy_phi, dt_);

        auto now = get_clock()->now();
        appendPath(truth_path_, truth_state_, now);
        appendPath(noisy_path_, noisy_state_, now);

        truth_path_pub_->publish(truth_path_);
        noisy_path_pub_->publish(noisy_path_);

        sim_time_ += dt_;
    }

    RobotState updateKinematics(RobotState s, double v, double phi, double dt)
    {
        double omega = (v / wheelbase_) * std::tan(phi);
        s.x += v * std::cos(s.theta) * dt;
        s.y += v * std::sin(s.theta) * dt;
        s.theta += omega * dt;
        return s;
    }

    void appendPath(nav_msgs::msg::Path & path, const RobotState & s, const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "odom";
        ps.pose.position.x = s.x;
        ps.pose.position.y = s.y;
        ps.pose.orientation.z = std::sin(s.theta / 2.0);
        ps.pose.orientation.w = std::cos(s.theta / 2.0);
        path.poses.push_back(ps);
    }

    double wheelbase_{1.5};
    double dt_{0.05};
    double sim_time_{0.0};
    double noise_level_{0.0};

    RobotState truth_state_, noisy_state_;
    nav_msgs::msg::Path truth_path_, noisy_path_;

    std::default_random_engine generator_;
    std::normal_distribution<double> noise_dist_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr truth_path_pub_, noisy_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannNoiseNode>());
    rclcpp::shutdown();
    return 0;
}