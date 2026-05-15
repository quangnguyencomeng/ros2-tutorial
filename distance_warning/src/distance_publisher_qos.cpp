/*Publish /distance_reliable với QoS RELIABLE — 1 Hz
Publish /distance_best_effort với QoS BEST_EFFORT — 10 Hz
Cùng dữ liệu ngẫu nhiên 0.1–1.5 m
Log phân biệt rõ đang publish lên topic nào*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>
using namespace std; //thư viện tiêu chuẩn
// dùng để viết ms, s thay vì tính toán 
#include<chrono>
using namespace chrono_literals; //để dùng hậu tố ms hay s cho thời gian
/////

class DistancePublisherQoS : public rclcpp::Node
{
public:
  DistancePublisherQoS() : Node("distance_publisher_qos")
  {
    auto qos_reliable = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    auto qos_best_effort = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    // TODO: Tạo publisher reliable trên '/distance_reliable'
    // TODO: Tạo publisher best_effort trên '/distance_best_effort'
    // TODO: Timer 1 Hz → publishReliable()
    // TODO: Timer 100ms → publishBestEffort()
    //thay thế 10 (size tự đặt) bằng qos_reliable và qos_best_effort
    pub_reliable_ = this->create_publisher<std_msgs::msg::Float32>("distance_reliable",
    qos_reliable);
    pub_best_effort_ = this->create_publisher<std_msgs::msg::Float32>("distance_best_effort",
    qos_best_effort);
    timer_reliable_ = this->create_wall_timer(1s, bind(&DistancePublisherQoS::publishReliable, this));
    timer_best_effort_ = this->create_wall_timer(100ms, bind(&DistancePublisherQoS::publishBestEffort,this));

  }

private:
  float randomDistance()
  {
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<float> dist(0.1f, 1.5f);
    return dist(rng);
  }

  void publishReliable()
  {
    auto msg = std_msgs::msg::Float32();
    // TODO: msg.data = randomDistance(), publish, log "[RELIABLE 1Hz]"
    msg.data = randomDistance();
    pub_reliable_ -> publish(msg);
    RCLCPP_INFO(this->get_logger(), "[RELIABLE 1Hz] Publishing: '%.2f' m", msg.data);
  }

  void publishBestEffort()
  {
    auto msg = std_msgs::msg::Float32();
    // TODO: msg.data = randomDistance(), publish, log "[BEST_EFFORT 10Hz]"
    msg.data = randomDistance();
    pub_best_effort_ -> publish(msg);
    RCLCPP_INFO(this-> get_logger(), "[BEST_EFFORT 10Hz] Publishing: '%.2f'", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_reliable_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_best_effort_;
  rclcpp::TimerBase::SharedPtr timer_reliable_;
  rclcpp::TimerBase::SharedPtr timer_best_effort_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistancePublisherQoS>());
  rclcpp::shutdown();
  return 0;
}