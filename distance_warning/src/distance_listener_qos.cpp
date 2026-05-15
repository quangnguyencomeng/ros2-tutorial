/*Subscribe /distance_reliable với QoS RELIABLE
Subscribe /distance_best_effort với QoS BEST_EFFORT
Đếm message nhận được từ mỗi topic
Mỗi 5 giây in bảng thống kê và reset counter
Khi nhận message từ /distance_reliable: kiểm tra ngưỡng và in cảnh báo (tái sử dụng logic từ distance_listener BTVN_01)*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
using namespace std;
using namespace rclcpp;

using placeholders::_1;// ghế trống cho parameter

class DistanceListenerQoS : public rclcpp::Node
{
public:
  DistanceListenerQoS()
  : Node("distance_listener_qos"), count_reliable_(0), count_best_effort_(0)
  {
    this->declare_parameter<double>("threshold", 0.5);

    auto qos_reliable = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::Reliable);

    auto qos_best_effort = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // TODO: Subscribe '/distance_reliable' với qos_reliable → reliableCallback
    // TODO: Subscribe '/distance_best_effort' với qos_best_effort → bestEffortCallback
    sub_reliable_ = this -> create_subscription<std_msgs::msg::Float32>("distance_reliable", qos_reliable, bind(&DistanceListenerQoS::reliableCallback, this, _1));
    sub_best_effort_ = this ->create_subscription<std_msgs::msg::Float32>("distance_best_effort",qos_best_effort, bind(&DistanceListenerQoS::bestEffortCallback, this, _1));
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&DistanceListenerQoS::printStats, this));
  }

private:
  void reliableCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    count_reliable_++;
    double threshold = this->get_parameter("threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "[RELIABLE]    %.2f m (total: %d)",
                msg->data, count_reliable_);
    // TODO: Nếu msg->data < threshold → RCLCPP_WARN cảnh báo
    if (msg->data < threshold){
        RCLCPP_WARN(this->get_logger(), "[distance_listener_qos]: Warning: Object too close! (%.2f m < threshold: %.2f m)", msg->data, threshold);
    }
  }

  void bestEffortCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    count_best_effort_++;
    RCLCPP_INFO(this->get_logger(), "[BEST_EFFORT] %.2f m (total: %d)",
                msg->data, count_best_effort_);
  }

  void printStats()
  {
    RCLCPP_INFO(this->get_logger(), "--- Stats (last 5s) ---");
    RCLCPP_INFO(this->get_logger(), "RELIABLE    : %d msg (expected ~5)",  count_reliable_);
    RCLCPP_INFO(this->get_logger(), "BEST_EFFORT : %d msg (expected ~50)", count_best_effort_);
    count_reliable_ = 0;
    count_best_effort_ = 0;
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_reliable_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_best_effort_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  int count_reliable_;
  int count_best_effort_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceListenerQoS>());
  rclcpp::shutdown();
  return 0;
}

//ros2 topic echo /distance_reliable
//ros2 topic echo /distance_best_effort
// ros2 topic echo /distance_best_effort --qos-reliability reliable