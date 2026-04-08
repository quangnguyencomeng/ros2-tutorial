/*Subscribe /distance_topic kiểu std_msgs/msg/Float32
Broadcast world → base_link: cố định tại gốc (0, 0, 0)
Broadcast base_link → sensor_link: x = distance, y = 0, z = 0
Broadcast mỗi lần nhận message từ /distance_topic*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class DistanceTfBroadcaster : public rclcpp::Node
{
public:
  DistanceTfBroadcaster() : Node("distance_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // TODO: Subscribe '/distance_topic' gọi broadcastCallback
    subscription_ = this -> create_subscription<std_msgs::msg::Float32>(
        "/distance_topic",
        10,
        std::bind(&DistanceTfBroadcaster::broadcastCallback, this, std::placeholders::_1)
    );
  }

private:
  void broadcastCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    auto now = this->get_clock()->now(); // lấy giờ hiện tại

    // --- world → base_link ---
    geometry_msgs::msg::TransformStamped t_base;
    t_base.header.stamp = now; //timestamp
    t_base.header.frame_id = "world"; //cha
    t_base.child_frame_id = "base_link"; //con
    t_base.transform.translation.x = 0.0;
    t_base.transform.translation.y = 0.0;
    t_base.transform.translation.z = 0.0;
    t_base.transform.rotation.w = 1.0; // quaternion (0,0,0,1) → không xoay. thiếu x,y,z nhưng vẫn hiểu default là 0 
    tf_broadcaster_->sendTransform(t_base); // gửi transform lên /tf

    // --- base_link → sensor_link ---
    geometry_msgs::msg::TransformStamped t_sensor;
    t_sensor.header.stamp = now;
    t_sensor.header.frame_id = "base_link";
    t_sensor.child_frame_id = "sensor_link";
    // TODO: Set translation.x = msg->data, y = 0, z = 0 //sensor nằm phía trước robot một khoảng = distance
    // TODO: Set rotation.w = 1.0
    t_sensor.transform.translation.x = msg -> data;
    t_sensor.transform.translation.y =0;
    t_sensor.transform.translation.z = 0;
    t_sensor.transform.rotation.w =1.0; // cái nào không khai báo mặc định 0
    tf_broadcaster_->sendTransform(t_sensor);

    RCLCPP_INFO(this->get_logger(),
      "Broadcasting sensor_link at x=%.2f m", msg->data);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

//  ros2 run distance_warning distance_publisher -> publisher gửi data lên /distance_topic
//  ros2 run distance_warning distance_tf_broadcaster -> broadcaster chờ ở /distance_topic và lấy thông tin, sau đó gọi broadcasterCallback
//  ros2 run tf2_ros tf2_echo base_link sensor_link -> cho biết toàn bộ thông tin của transform tại từng thời điểm (như vd này là chiều base_link -> sensor_link)
        /*  At time 1775565968.958721872
            - Translation: [0.887, 0.000, 0.000]
            - Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
            - Rotation: in RPY (radian) [0.000, -0.000, 0.000]
            - Rotation: in RPY (degree) [0.000, -0.000, 0.000]
            - Matrix:
              1.000  0.000  0.000  0.887
              0.000  1.000  0.000  0.000
              0.000  0.000  1.000  0.000
              0.000  0.000  0.000  1.000 */

// ros2 run tf2_tools view_frames // bản vẽ TF tree (pdf và txt)
        /*digraph G {
            "world" -> "base_link"[label=" Broadcaster: default_authority\nAverage rate: 1.25\nBuffer length: 4.0\nMost recent transform: 1775565550.958067\nOldest transform: 1775565546.958042\n"];
            "base_link" -> "sensor_link"[label=" Broadcaster: default_authority\nAverage rate: 1.25\nBuffer length: 4.0\nMost recent transform: 1775565550.958067\nOldest transform: 1775565546.958042\n"];
            edge [style=invis];
            subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";
            "Recorded at time: 1775565551.2654905"[ shape=plaintext ] ;
            }->"world";
            }*/