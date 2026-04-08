/*Lookup transform world → sensor_link mỗi 1 giây
Tính khoảng cách Euclidean từ world origin đến sensor_link
In khoảng cách tính được
Khi khoảng cách > tf_threshold (parameter, mặc định 1.0 m): gọi service /set_threshold với increase=false để tự động giảm threshold cảnh báo
Declare parameter tf_threshold mặc định 1.0*/

using namespace std;
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "distance_warning/srv/set_threshold.hpp"
#include <cmath>
#include<chrono>
using namespace chrono_literals; //để dùng hậu tố ms hay s cho thời gian
/////

using SetThreshold = distance_warning::srv::SetThreshold;

class DistanceTfListener : public rclcpp::Node
{
public:
  DistanceTfListener() : Node("distance_tf_listener")
  {
    this->declare_parameter<double>("tf_threshold", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    threshold_client_ = this->create_client<SetThreshold>("set_threshold");

    // TODO: Tạo timer 1 Hz gọi lookupAndCheck()
    timer_ = this->create_wall_timer(1s, bind(&DistanceTfListener::lookupAndCheck, this));

  }

private:
  void lookupAndCheck()
  {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        "world", "sensor_link", tf2::TimePointZero);

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;
      RCLCPP_INFO(this->get_logger(), "TF world -> sensor_link: x=%.2f, y=%.2f, z=%.2f", x, y, z);
      
      // TODO: Tính khoảng cách Euclidean: dist = sqrt(x² + y² + z²)
      // TODO: In khoảng cách
      // TODO: Nếu dist > tf_threshold → gọi callSetThreshold(false)

      double dist = sqrt(pow(x,2)+ pow(y,2) + pow(z,2));
      RCLCPP_INFO(this->get_logger(), "Distance: %.3f", dist);
      double tf_threshold =this -> get_parameter("tf_threshold").as_double();
      if (dist > tf_threshold){
        callSetThreshold(false);
      }

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  void callSetThreshold(bool increase)
  {
    if (!threshold_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(), "set_threshold service not available");
      return;
    }
    auto request = std::make_shared<SetThreshold::Request>();
    request->increase = increase;
    threshold_client_->async_send_request(request,
      [this](rclcpp::Client<SetThreshold>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(),
            "Auto-adjusted threshold to %.2f m", response->new_threshold);
        }
      });
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<SetThreshold>::SharedPtr threshold_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceTfListener>());
  rclcpp::shutdown();
  return 0;
}

