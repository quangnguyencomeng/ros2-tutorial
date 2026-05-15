#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MinimalLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MinimalLifecycleNode() : LifecycleNode("minimal_lifecycle_node") {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    pub_ = this->create_publisher<std_msgs::msg::String>("/status", rclcpp::QoS(10));
    timer_ = create_wall_timer(1s, [this]() { publish_status(); });
    timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    // Bắt buộc gọi parent để chuyển state đúng cách
    LifecycleNode::on_activate(state);
    pub_->on_activate();
    // FIX 3: Tạo lại timer thay vì reset() — reset() không resume timer đã cancel
    timer_ = create_wall_timer(1s, [this]() { publish_status(); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    LifecycleNode::on_deactivate(state);
    pub_->on_deactivate();
    timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    count_ = 0;
    pub_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return CallbackReturn::SUCCESS;
  }

private:
  void publish_status()
  {
    if (!pub_->is_activated()) return;
    auto msg = std_msgs::msg::String();
    msg.data = "Node is active - " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: %s", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalLifecycleNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}