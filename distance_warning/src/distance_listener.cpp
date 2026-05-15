// nhiệm vụ:  
  // + subscribe vào topic distance_topic với kiểu dữ liệu std_msgs/msg/Float32
  // + khai báo cấu hình para threshold (giá trị mặc định là 0.5m)
  // + so sánh với khoảng cách từ topic để: 
    // nhỏ hơn: warning
    // lớn hơn hoặc bằng: báo an toàn 

#include <memory> // Để dùng std::shared_ptr


#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std;
using namespace rclcpp;

using placeholders::_1;// ghế trống cho parameter

class DistanceListener:public Node{

private:
  // Khai báo biến thành viên để giữ cho bộ phận lắng nghe không bị biến mất
  Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;


  // Hàm này tự động chạy MỖI KHI có một tin nhắn mới bay tới
  // 'msg' chính là con trỏ (SharedPtr) chứa gói dữ liệu đó
  void ListenerCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    //lấy giá trị của threshold
    // lấy trong hàm callback thì nếu có thay đổi tham số thì sẽ cập nhật luôn
    double threshold = this->get_parameter("threshold").as_double();
    float distance = msg->data;

    if (distance < threshold){
      RCLCPP_WARN (this->get_logger(),"Warning: Object too close! (%.2f m < threshold: %.2f m)", distance, threshold);
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Distance received: %.2f m", distance);

    }
  }
public:
  // constructor 
  DistanceListener() : Node("distance_listener"){

    // 1. khai bao para
    this->declare_parameter("threshold", 0.5); //constructor là cái declare threshold
    

    //2. Khoi tao sub
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "distance_topic", 10, bind(&DistanceListener::ListenerCallback, this,_1)
    );

    //3. in thong bao da khoi tao listener xong
    RCLCPP_INFO(this->get_logger(), "Distance Subscriber Node created");
  }
  
};

int main(int argc,char * argv[]){
  init(argc, argv);
  spin(make_shared<DistanceListener>());

  shutdown();
  return 0;
}

//ros2 run distance_warning set_threshold_service
//ros2 service call /set_threshold distance_warning/srv/SetThreshold "{increase: true}"
