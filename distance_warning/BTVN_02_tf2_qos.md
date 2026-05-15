# BTVN_02 — Mở rộng `distance_warning`

> Tiếp tục từ BTVN_01 | Language: C++ (rclcpp) | ROS2 Humble

---

## Tổng quan

BTVN_02 **không tạo package mới**. Mở rộng thẳng vào package `distance_warning` từ BTVN_01 bằng cách thêm 4 node mới. Sau khi hoàn thành, hệ thống sẽ có 9 node hoạt động cùng nhau.

### Cấu trúc package sau khi hoàn thành

```
distance_warning/
├── src/
│   ├── distance_publisher.cpp          ← BTVN_01 (giữ nguyên)
│   ├── distance_listener.cpp           ← BTVN_01 (giữ nguyên)
│   ├── set_threshold_service.cpp       ← BTVN_01 (giữ nguyên)
│   ├── distance_action_server.cpp      ← BTVN_01 (giữ nguyên)
│   ├── distance_action_client.cpp      ← BTVN_01 (giữ nguyên)
│   ├── distance_tf_broadcaster.cpp     ← BTVN_02 mới 
│   ├── distance_tf_listener.cpp        ← BTVN_02 mới 
│   ├── distance_publisher_qos.cpp      ← BTVN_02 mới 
│   └── distance_listener_qos.cpp       ← BTVN_02 mới 
├── action/
│   └── CheckDistance.action            ← BTVN_01 (giữ nguyên)
├── srv/
│   └── SetThreshold.srv                ← BTVN_01 (giữ nguyên)
├── launch/
│   ├── distance_warning.launch.py      ← BTVN_01 (giữ nguyên)
│   └── distance_warning_full.launch.py ← BTVN_02 mới 
├── CMakeLists.txt                      ← cập nhật thêm dependencies
└── package.xml                         ← cập nhật thêm dependencies
```

---

## Bước 0 — Cập nhật dependencies

### `package.xml` — thêm vào

```xml
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>geometry_msgs</depend>
```

### `CMakeLists.txt` — thêm vào phần `find_package`

```cmake
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
```

---

## Kiến thức nền

### TF2

TF2 quản lý quan hệ không gian giữa các coordinate frame. Mỗi frame kết nối với frame cha qua một transform (vị trí + rotation).

```
world
  └── base_link          (vị trí robot)
        └── sensor_link  (vị trí cảm biến so với robot)
```

- **Broadcaster** — publish transform lên `/tf`
- **Listener** — đọc và tính transform giữa 2 frame bất kỳ

Trong bài này, giá trị `distance` từ `/distance_topic` sẽ được dùng để di chuyển `sensor_link` — khoảng cách đo được càng lớn thì `sensor_link` càng xa `base_link`.

### QoS

QoS kiểm soát độ tin cậy khi truyền message.

| Policy | Reliability | Dùng khi |
|--------|-------------|----------|
| `RELIABLE` | Đảm bảo nhận đủ, retry nếu mất | Data quan trọng, tần suất thấp |
| `BEST_EFFORT` | Ưu tiên tốc độ, chấp nhận mất packet | Sensor stream tần suất cao |

>  Publisher và Subscriber phải **compatible** QoS. `RELIABLE` subscriber **không nhận** được message từ `BEST_EFFORT` publisher.

---

## Phần 1 — TF2

### 1.1 Node `distance_tf_broadcaster`

**File:** `src/distance_tf_broadcaster.cpp`

**Yêu cầu:**
- Subscribe `/distance_topic` kiểu `std_msgs/msg/Float32`
- Broadcast `world → base_link`: cố định tại gốc `(0, 0, 0)`
- Broadcast `base_link → sensor_link`: `x = distance`, `y = 0`, `z = 0`
- Broadcast mỗi lần nhận message từ `/distance_topic`

**Code khung:**

```cpp
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
  }

private:
  void broadcastCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    auto now = this->get_clock()->now();

    // --- world → base_link ---
    geometry_msgs::msg::TransformStamped t_base;
    t_base.header.stamp = now;
    t_base.header.frame_id = "world";
    t_base.child_frame_id = "base_link";
    t_base.transform.translation.x = 0.0;
    t_base.transform.translation.y = 0.0;
    t_base.transform.translation.z = 0.0;
    t_base.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t_base);

    // --- base_link → sensor_link ---
    geometry_msgs::msg::TransformStamped t_sensor;
    t_sensor.header.stamp = now;
    t_sensor.header.frame_id = "base_link";
    t_sensor.child_frame_id = "sensor_link";
    // TODO: Set translation.x = msg->data, y = 0, z = 0
    // TODO: Set rotation.w = 1.0
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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_tf_broadcaster src/distance_tf_broadcaster.cpp)
ament_target_dependencies(distance_tf_broadcaster
  rclcpp std_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS distance_tf_broadcaster DESTINATION lib/${PROJECT_NAME})
```

**Kiểm tra:**

```bash
ros2 run tf2_ros tf2_echo base_link sensor_link
ros2 run tf2_tools view_frames
```

**Expected output:**

```
[INFO] [distance_tf_broadcaster]: Broadcasting sensor_link at x=0.82 m
[INFO] [distance_tf_broadcaster]: Broadcasting sensor_link at x=0.34 m

# tf2_echo:
At time 1234567890.0
- Translation: [0.820, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

---

### 1.2 Node `distance_tf_listener`

**File:** `src/distance_tf_listener.cpp`

**Yêu cầu:**
- Lookup transform `world → sensor_link` mỗi **1 giây**
- Tính khoảng cách Euclidean từ world origin đến `sensor_link`
- In khoảng cách tính được
- Khi khoảng cách `> tf_threshold` (parameter, mặc định `1.0 m`): gọi service `/set_threshold` với `increase=false` để tự động giảm threshold cảnh báo
- Declare parameter `tf_threshold` mặc định `1.0`

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "distance_warning/srv/set_threshold.hpp"
#include <cmath>

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

      // TODO: Tính khoảng cách Euclidean: dist = sqrt(x² + y² + z²)
      // TODO: In khoảng cách
      // TODO: Nếu dist > tf_threshold → gọi callSetThreshold(false)

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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_tf_listener src/distance_tf_listener.cpp)
ament_target_dependencies(distance_tf_listener
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
rosidl_target_interfaces(distance_tf_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS distance_tf_listener DESTINATION lib/${PROJECT_NAME})
```

**Expected output:**

```
[INFO] [distance_tf_listener]: TF distance world->sensor_link: 0.73 m
[INFO] [distance_tf_listener]: TF distance world->sensor_link: 1.21 m
[WARN] [distance_tf_listener]: Sensor out of range! 1.21 m > tf_threshold 1.00 m
[INFO] [distance_tf_listener]: Auto-adjusted threshold to 0.40 m
```

---

## Phần 2 — QoS

### 2.1 Node `distance_publisher_qos`

**File:** `src/distance_publisher_qos.cpp`

**Yêu cầu:**
- Publish `/distance_reliable` với QoS `RELIABLE` — **1 Hz**
- Publish `/distance_best_effort` với QoS `BEST_EFFORT` — **10 Hz**
- Cùng dữ liệu ngẫu nhiên `0.1–1.5 m`
- Log phân biệt rõ đang publish lên topic nào

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

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
  }

  void publishBestEffort()
  {
    auto msg = std_msgs::msg::Float32();
    // TODO: msg.data = randomDistance(), publish, log "[BEST_EFFORT 10Hz]"
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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_publisher_qos src/distance_publisher_qos.cpp)
ament_target_dependencies(distance_publisher_qos rclcpp std_msgs)
install(TARGETS distance_publisher_qos DESTINATION lib/${PROJECT_NAME})
```

---

### 2.2 Node `distance_listener_qos`

**File:** `src/distance_listener_qos.cpp`

**Yêu cầu:**
- Subscribe `/distance_reliable` với QoS `RELIABLE`
- Subscribe `/distance_best_effort` với QoS `BEST_EFFORT`
- Đếm message nhận được từ mỗi topic
- Mỗi **5 giây** in bảng thống kê và reset counter
- Khi nhận message từ `/distance_reliable`: kiểm tra ngưỡng và in cảnh báo (tái sử dụng logic từ `distance_listener` BTVN_01)

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_listener_qos src/distance_listener_qos.cpp)
ament_target_dependencies(distance_listener_qos rclcpp std_msgs)
install(TARGETS distance_listener_qos DESTINATION lib/${PROJECT_NAME})
```

**Expected output:**

```
[RELIABLE]    0.73 m (total: 1)
[BEST_EFFORT] 1.21 m (total: 1)
[BEST_EFFORT] 0.44 m (total: 2)
[RELIABLE]    0.28 m (total: 2)
[WARN] [distance_listener_qos]: Warning: Object too close! (0.28 m < threshold: 0.50 m)
--- Stats (last 5s) ---
RELIABLE    : 5 msg (expected ~5)
BEST_EFFORT : 48 msg (expected ~50)
```

### 2.3 Thử nghiệm QoS incompatible

```bash
# Cố tình subscribe BEST_EFFORT topic bằng RELIABLE subscriber
ros2 topic echo /distance_best_effort --qos-reliability reliable

# Quan sát QoS của từng topic
ros2 topic info /distance_reliable --verbose
ros2 topic info /distance_best_effort --verbose
```

ROS2 sẽ in warning:
```
New subscription discovered on topic '/distance_best_effort',
requesting incompatible QoS. No messages will be sent to it.
```

---

## Launch File

**File:** `launch/distance_warning_full.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.5',
        description='Distance warning threshold in meters'
    )
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        threshold_arg,

        # ── BTVN_01 ────────────────────────────────────
        Node(package='distance_warning',
             executable='distance_publisher',
             name='distance_publisher', output='screen'),
        Node(package='distance_warning',
             executable='distance_listener',
             name='distance_listener', output='screen',
             parameters=[{'threshold': threshold}]),
        Node(package='distance_warning',
             executable='set_threshold_service',
             name='set_threshold_service', output='screen',
             parameters=[{'threshold': threshold}]),
        Node(package='distance_warning',
             executable='distance_action_server',
             name='distance_action_server', output='screen',
             parameters=[{'threshold': threshold}]),

        # ── BTVN_02 ────────────────────────────────────
        Node(package='distance_warning',
             executable='distance_tf_broadcaster',
             name='distance_tf_broadcaster', output='screen'),
        Node(package='distance_warning',
             executable='distance_tf_listener',
             name='distance_tf_listener', output='screen',
             parameters=[{'tf_threshold': 1.0}]),
        Node(package='distance_warning',
             executable='distance_publisher_qos',
             name='distance_publisher_qos', output='screen'),
        Node(package='distance_warning',
             executable='distance_listener_qos',
             name='distance_listener_qos', output='screen',
             parameters=[{'threshold': threshold}]),
    ])
```

**Build và chạy:**

```bash
cd ~/ros2_ws
colcon build --packages-select distance_warning
source install/setup.bash

ros2 launch distance_warning distance_warning_full.launch.py
```

---

## Tiêu chí chấp nhận

| # | Tiêu chí | Kiểm tra bằng |
|---|----------|---------------|
| 1 | Package build không có warning sau khi thêm dependencies | `colcon build` |
| 2 | `distance_tf_broadcaster` broadcast đủ 2 frame lên `/tf` | `ros2 run tf2_ros tf2_echo base_link sensor_link` |
| 3 | `sensor_link` x thay đổi theo giá trị `/distance_topic` | So sánh log publisher và tf2_echo |
| 4 | `distance_tf_listener` tính đúng khoảng cách từ TF | Quan sát log |
| 5 | Khi sensor > `tf_threshold`, tự động gọi `/set_threshold` | Quan sát log cả 2 node |
| 6 | `/distance_reliable` publish đúng 1 Hz | `ros2 topic hz /distance_reliable` |
| 7 | `/distance_best_effort` publish đúng 10 Hz | `ros2 topic hz /distance_best_effort` |
| 8 | Bảng stats in đúng số lượng sau 5 giây | Quan sát log |
| 9 | QoS incompatible test → warning xuất hiện | `ros2 topic echo ... --qos-reliability reliable` |
| 10 | Launch file full khởi động đủ 8 node | `rqt_graph` |
