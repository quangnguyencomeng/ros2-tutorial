# Hướng Dẫn Tìm Hiểu Package NAV2 trong ROS2
**Môi trường thực hành:** TurtleBot3 Burger trên Gazebo Simulation
**Ngôn ngữ lập trình chính:** C++

---

## 1. Kiến Thức Cần Tìm Hiểu

### 1.1 Tổng quan về Navigation Stack (NAV2)

- NAV2 là gì và vai trò trong hệ thống robot tự hành
- Kiến trúc tổng thể của NAV2: các thành phần chính và cách chúng giao tiếp
- Lifecycle nodes và tại sao NAV2 sử dụng chúng

### 1.2 Bản Đồ và Định Vị (Map & Localization)

- **Occupancy Grid Map:** cấu trúc dữ liệu, ý nghĩa các giá trị (free / occupied / unknown)
- **SLAM (Simultaneous Localization and Mapping):**
  - Khái niệm và bài toán SLAM
  - Sử dụng `slam_toolbox` để tạo bản đồ
- **AMCL (Adaptive Monte Carlo Localization):**
  - Nguyên lý hoạt động của particle filter
  - Các tham số quan trọng: `min_particles`, `max_particles`, `laser_model_type`
  - Topic `/initialpose` và cách đặt vị trí ban đầu

### 1.3 Costmap

- Costmap 2D là gì và tại sao cần thiết
- **Global Costmap** vs **Local Costmap** -- mục đích và phạm vi
- Các layer trong costmap:
  - `StaticLayer` -- dữ liệu từ bản đồ tĩnh
  - `ObstacleLayer` -- vật cản từ sensor (LiDAR)
  - `InflationLayer` -- vùng đệm an toàn quanh vật cản
- Tham số quan trọng: `resolution`, `robot_radius`, `inflation_radius`, `cost_scaling_factor`

### 1.4 Lập Kế Hoạch Đường Đi (Path Planning)

- **Global Planner** -- tìm đường tổng thể từ điểm đầu đến đích:
  - `NavFn` (Dijkstra / A*)
  - `Smac Planner 2D`
  - `Theta* Planner`
- **Local Planner / Controller** -- điều khiển robot bám theo đường:
  - `DWB (Dynamic Window Approach B)` -- nguyên lý và tham số
  - `RPP (Regulated Pure Pursuit)` -- phù hợp cho differential drive
- Sự phối hợp giữa global planner và local controller

### 1.5 Recovery Behaviors

- Khi nào robot kích hoạt recovery behavior?
- Các hành vi phục hồi mặc định:
  - `ClearCostmapRecovery` -- xóa costmap khi bị kẹt
  - `SpinRecovery` -- xoay tại chỗ
  - `BackUpRecovery` -- lùi lại
  - `WaitRecovery`
- Behavior Tree (BT) và vai trò trong việc điều phối navigation

### 1.6 Behavior Tree và NAV2 BT Navigator

- Khái niệm Behavior Tree: Sequence, Fallback, Decorator, Action, Condition
- Cách NAV2 sử dụng BT để điều phối toàn bộ quá trình navigation
- Đọc hiểu file XML mô tả Behavior Tree mặc định của NAV2
- Công cụ `Groot2` để visualize và chỉnh sửa BT

### 1.7 Action Server và Navigation Goals trong C++

- Action interface trong ROS2: `action_msgs`, `rclcpp_action`
- Action `/navigate_to_pose` -- gửi mục tiêu và nhận feedback
- Action `/navigate_through_poses` -- navigation qua nhiều điểm
- Action `/follow_waypoints` -- tuần tự đi qua danh sách waypoint
- Viết action client bằng C++ với `rclcpp_action::create_client`
- Xử lý goal response callback, feedback callback, và result callback

### 1.8 TF2 và Robot Frames

- Cây TF trong NAV2: `map -> odom -> base_link -> sensor_frames`
- Sự khác biệt giữa frame `map` và frame `odom`
- Odometry drift và tại sao cần AMCL để hiệu chỉnh
- Công cụ `tf2_tools`, `rqt_tf_tree`, `rviz2`

### 1.9 Cấu Hình và Tùy Chỉnh NAV2

- Cấu trúc file `nav2_params.yaml`
- Launch file của NAV2: `bringup_launch.py`
- Tùy chỉnh planner và controller thông qua plugin
- Plugin architecture trong NAV2: `pluginlib` và cách đăng ký plugin C++

---

## 2. Tài Liệu Tham Khảo

### Tài liệu chính thức

| Tài liệu | Liên kết |
|---|---|
| NAV2 Documentation (Official) | https://docs.nav2.org |
| NAV2 GitHub Repository | https://github.com/ros-navigation/navigation2 |
| ROS2 Documentation | https://docs.ros.org |
| TurtleBot3 e-Manual | https://emanual.robotis.com/docs/en/platform/turtlebot3/overview |
| rclcpp_action API | https://docs.ros2.org/latest/api/rclcpp_action |

### Video và khóa học

| Tài liệu | Liên kết |
|---|---|
| NAV2 Tutorial Series - The Construct | https://www.youtube.com/@TheConstruct |
| Articulated Robotics - NAV2 Playlist | https://www.youtube.com/@ArticulatedRobotics |
| ROS2 Navigation - Robotics Back-End | https://www.youtube.com/@RoboticsBackEnd |

### Packages liên quan cần tìm hiểu thêm

- `slam_toolbox` -- https://github.com/SteveMacenski/slam_toolbox
- `BehaviorTree.CPP` -- thư viện BT nền tảng của NAV2
- `Groot2` -- visualizer cho Behavior Tree
- `pluginlib` -- hệ thống plugin của ROS2

---

## 3. Bài Luyện Tập / Tìm Hiểu Rõ Hơn

**Lưu ý chung:**
- Tất cả bài lab thực hiện trên **TurtleBot3 Burger với Gazebo simulation**
- Môi trường khuyến dùng: `turtlebot3_world` hoặc `turtlebot3_house`
- Cài đặt cần thiết trước khi bắt đầu:

```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox
export TURTLEBOT3_MODEL=burger
```

---

### Lab 1 -- Khởi Động và Quan Sát Hệ Thống NAV2

**Mục tiêu:** Hiểu các node, topic, và action server trong NAV2 đang chạy.

**Khởi chạy môi trường:**

```bash
# Terminal 1 -- Gazebo simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 -- NAV2 (cần có bản đồ trước, xem Lab 2)
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=$HOME/maps/turtlebot3_world.yaml
```

**Yêu cầu:**

1. Dùng `ros2 node list` liệt kê tất cả node đang chạy, xác định node nào thuộc NAV2
2. Dùng `ros2 topic list` và `ros2 topic info <topic>` phân tích các topic chính:
   - `/scan` -- dữ liệu LiDAR
   - `/odom` -- odometry
   - `/cmd_vel` -- lệnh vận tốc
   - `/map` -- bản đồ
   - `/amcl_pose` -- vị trí ước tính của AMCL
3. Liệt kê các action server đang chạy:
   ```bash
   ros2 action list
   ros2 action info /navigate_to_pose
   ```
4. Dùng `rqt_graph` vẽ sơ đồ giao tiếp giữa các node và chụp ảnh
5. Mở RViz2, bật hiển thị: Global Costmap, Local Costmap, Path, Particle Cloud (AMCL)

**Câu hỏi thảo luận:**
- Node nào publish `/cmd_vel`? Node nào subscribe nó?
- Topic `/amcl_pose` và `/odom` khác nhau như thế nào về `frame_id` và mục đích?
- Tại sao có 2 costmap riêng biệt với kích thước và tần số cập nhật khác nhau?

---

### Lab 2 -- Tạo Bản Đồ với SLAM Toolbox

**Mục tiêu:** Thực hành tạo bản đồ và lưu bản đồ cho TurtleBot3 trong `turtlebot3_world`.

**Khởi chạy:**

```bash
# Terminal 1 -- Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 -- SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3 -- Điều khiển bằng bàn phím
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Yêu cầu:**

1. Điều khiển TurtleBot3 di chuyển chậm và đều để LiDAR quét toàn bộ môi trường
2. Quan sát bản đồ được xây dựng từng bước trên RViz2 (topic `/map`)
3. Khi bản đồ hoàn chỉnh, lưu lại:
   ```bash
   mkdir -p ~/maps
   ros2 run nav2_map_server map_saver_cli -f ~/maps/turtlebot3_world
   ```
4. Mở và giải thích nội dung 2 file vừa lưu:
   - `turtlebot3_world.pgm`: ý nghĩa giá trị pixel (0 = occupied, 254 = free, 205 = unknown)
   - `turtlebot3_world.yaml`: các trường `resolution`, `origin`, `occupied_thresh`, `free_thresh`
5. Thử thay đổi tham số `resolution` trong SLAM (mặc định 0.05 m/pixel so với 0.025 m/pixel) và quan sát sự khác biệt chất lượng bản đồ

**Câu hỏi thảo luận:**
- Tham số `resolution` ảnh hưởng gì đến bộ nhớ và độ chính xác navigation?
- Điều gì xảy ra nếu robot di chuyển quá nhanh hoặc quay vòng quá gấp khi mapping?

---

### Lab 3 -- Phân Tích Costmap và Inflation Layer

**Mục tiêu:** Hiểu cách costmap được xây dựng và tác động đến đường đi.

**Yêu cầu:**

1. Launch NAV2 với bản đồ đã tạo ở Lab 2:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
     use_sim_time:=True map:=$HOME/maps/turtlebot3_world.yaml
   ```

2. Trong RViz2, bật hiển thị cả `Global Costmap` và `Local Costmap`

3. Tìm và sao chép file params mặc định ra thư mục làm việc:
   ```bash
   cp /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml ~/my_nav2_params.yaml
   ```

4. Tùy chỉnh `inflation_radius` trong cả 2 costmap, thử 3 giá trị: 0.2, 0.55, 1.0:
   ```yaml
   inflation_layer:
     plugin: "nav2_costmap_2d::InflationLayer"
     cost_scaling_factor: 3.0
     inflation_radius: 0.55   # Thay đổi giá trị này
   ```

5. Với mỗi giá trị, gửi goal qua RViz2 và ghi nhận:
   - Hình dạng vùng màu trên costmap
   - Robot đi sát hay xa vật cản
   - Liệt kê trường hợp nào robot không tìm được đường

**Ghi chép:** Chụp ảnh màn hình RViz2 ở mỗi mức `inflation_radius` để đưa vào báo cáo so sánh.

**Câu hỏi thảo luận:**
- `cost_scaling_factor` ảnh hưởng thế nào đến gradient của vùng làm phát?
- Nếu `inflation_radius` lớn hơn kích thước hành lang trong môi trường, điều gì xảy ra?

---

### Lab 4 -- Gửi Navigation Goal bằng C++

**Mục tiêu:** Viết node C++ sử dụng `rclcpp_action` để gửi goal đến NAV2 và xử lý phản hồi.

**Tạo package mới:**

```bash
cd ~/ros2_ws/src
ros2 pkg create nav2_goal_sender --build-type ament_cmake \
  --dependencies rclcpp rclcpp_action nav2_msgs geometry_msgs
```

**Yêu cầu -- viết file `src/goal_sender.cpp`:**

1. Tạo class `GoalSenderNode` kế thừa `rclcpp::Node`
2. Khởi tạo `rclcpp_action::Client<NavigateToPose>` cho action `/navigate_to_pose`
3. Hàm `send_goal(double x, double y, double yaw)` thực hiện:
   - Đợi action server sẵn sàng (`wait_for_action_server`)
   - Tạo `NavigateToPose::Goal` với `PoseStamped` có `frame_id = "map"`
   - Chuyển `yaw` thành quaternion: `z = sin(yaw/2)`, `w = cos(yaw/2)`
   - Gửi goal với `async_send_goal` kèm 3 callback
4. Implement các callback:
   - `goal_response_callback`: in ra goal có được chấp nhận hay bị từ chối
   - `feedback_callback`: in ra `current_pose` và `estimated_time_remaining`
   - `result_callback`: xử lý SUCCEEDED, ABORTED, CANCELED

**Template khởi đầu:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalSenderNode : public rclcpp::Node
{
public:
  GoalSenderNode() : Node("goal_sender")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void SendSoal(double x, double y, double yaw)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
      RCLCPP_ERROR(get_logger(), "Action server không sẵn sàng!");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = now();
    goal_msg.pose.pose.position.x  = x;
    goal_msg.pose.pose.position.y  = y;
    // TODO: Tính quaternion từ yaw và gán vào orientation

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // TODO: Gán GoalResponseCallback, FeedbackCallback, ResultCallback

    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "Đã gửi goal: (%.2f, %.2f, %.2f)", x, y, yaw);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  void GoalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    // TODO
  }

  void FeedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // TODO: In vị trí hiện tại và estimated_time_remaining
  }

  void ResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    // TODO: Xử lý SUCCEEDED, ABORTED, CANCELED
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalSenderNode>();
  node->send_goal(1.5, -0.5, 0.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

**Kiểm tra:** Build bằng `colcon build`, chạy node và quan sát trên RViz2 xem robot có đi đến đúng tọa độ không.

---

### Lab 5 -- Tùy Chỉnh Planner và Controller

**Mục tiêu:** Hiểu cách thay thế plugin trong NAV2 và quan sát sự khác biệt.

**Yêu cầu:**

1. Sử dụng file `~/my_nav2_params.yaml` từ Lab 3

2. Thay đổi **global planner** từ `NavFn` sang `SmacPlanner2D`:
   ```yaml
   planner_server:
     ros__parameters:
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_smac_planner/SmacPlanner2D"
         tolerance: 0.5
         use_astar: true
         allow_unknown: true
   ```

3. Thay đổi **local controller** từ `DWB` sang `RegulatedPurePursuit`:
   ```yaml
   controller_server:
     ros__parameters:
       controller_plugins: ["FollowPath"]
       FollowPath:
         plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
         desired_linear_vel: 0.2
         lookahead_dist: 0.6
         min_lookahead_dist: 0.3
         max_lookahead_dist: 0.9
   ```

4. Launch NAV2 với params tùy chỉnh:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
     use_sim_time:=True \
     map:=$HOME/maps/turtlebot3_world.yaml \
     params_file:=$HOME/my_nav2_params.yaml
   ```

5. Gửi goal đến cùng một đích với 2 tổ hợp planner/controller khác nhau, chụp ảnh RViz2:
   - Hình dạng đường đi (path) có khác nhau không?
   - Cách robot bám theo đường có khác nhau không?

**Câu hỏi thảo luận:**
- `NavFn` (Dijkstra) và `SmacPlanner2D` (A*) khác nhau về gì về mặt thuật toán?
- `RPP` có ưu điểm gì so với `DWB` khi chạy trên TurtleBot3 trong môi trường này?

---

### Lab 6 -- Behavior Tree (Nâng cao)

**Mục tiêu:** Đọc hiểu và chỉnh sửa Behavior Tree mặc định của NAV2.

**Yêu cầu:**

1. Tìm file BT XML mặc định:
   ```bash
   find /opt/ros/humble -name "navigate_to_pose_w_replanning_and_recovery.xml" 2>/dev/null
   ```

2. Cài đặt Groot2 và mở file BT để visualize cấu trúc cây

3. Đọc hiểu luồng hoạt động và trả lời:
   - `ComputePathToPose` nằm ở vị trí nào trong cây BT?
   - Khi nào node `RoundRobin` kích hoạt các recovery behavior?
   - `Sequence` và `Fallback` xuất hiện ở những bước nào, với ý nghĩa gì?

4. Chỉnh sửa file XML: thêm hành vi `Wait` 2 giây trước khi thực hiện recovery `Spin`:
   ```xml
   <Fallback>
     <Sequence>
       <IsStuck/>
       <Wait wait_duration="2.0"/>
       <Spin spin_dist="1.57"/>
     </Sequence>
   </Fallback>
   ```

5. Launch NAV2 với BT tùy chỉnh bằng cách thêm vào `nav2_params.yaml`:
   ```yaml
   bt_navigator:
     ros__parameters:
       default_nav_to_pose_bt_xml: "/home/<user>/my_custom_bt.xml"
   ```
   Kích hoạt recovery bằng cách đặt robot vào vị trí bị chặn, quan sát xem robot có Wait trước khi Spin không.

---

## 4. Bài Tập Về Nhà

**Yêu cầu nộp bài:** Báo cáo PDF gồm mô tả giải pháp, toàn bộ source code, ảnh chụp màn hình kết quả, nhận xét.

---

### Bài 1 -- Patrol Robot

**Đề bài:** Viết ROS2 node **bằng C++** để TurtleBot3 tự động tuần tra qua 4 điểm trong bản đồ `turtlebot3_world`, lặp lại theo vòng vô tận.

**Yêu cầu cụ thể:**
- Dùng `rclcpp_action::Client<NavigateToPose>` để gửi goal tới `/navigate_to_pose`
- Dừng lại 3 giây tại mỗi điểm (dùng `rclcpp::sleep_for` hoặc `rclcpp::TimerBase`)
- In `RCLCPP_INFO` log khi đến được mỗi điểm và khi bắt đầu đi đến điểm tiếp theo
- Nếu goal trả về `ABORTED`, log cảnh báo và chuyển sang điểm tiếp theo trong danh sách
- Xử lý graceful shutdown khi nhận `Ctrl+C` với `rclcpp::on_shutdown`

**Gợi ý cấu trúc:**

```cpp
class PatrolNode : public rclcpp::Node
{
public:
  PatrolNode();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp_action::Client<NavigateToPose>::SharedPtr m_nav_client;

  // Danh sách waypoint: {x, y, yaw}
  std::vector<std::tuple<double, double, double>> m_waypoints;
  size_t m_current_index{0};

  void SendNextGoal();
  void ResultCallback(const GoalHandleNav::WrappedResult & result);
};
```

**Tiêu chí đánh giá:**
- Robot đi đúng thứ tự 4 waypoint và lặp lại đúng cách
- Xử lý được trường hợp goal trả về ABORTED
- Code có comment giải thích rõ ràng từng phần logic
- Báo cáo có video quay màn hình Gazebo + RViz2 (ít nhất 1 vòng hoàn chỉnh)

---

### Bài 2 -- So Sánh Planner qua Metrics

**Đề bài:** Viết chương trình C++ tự động thí nghiệm và thu thập số liệu so sánh 2 global planner (`NavFn` và `SmacPlanner2D`) trên cùng một loạt goal trong `turtlebot3_world`.

**Yêu cầu cụ thể:**

1. Viết node C++ thực hiện:
   - Gửi goal từ A đến B, ghi lại thời gian hoàn thành từ lúc gửi goal đến khi nhận SUCCEEDED
   - Lặp lại 5 lần với cùng goal đó cho mỗi planner
   - Tính tổng quãng đường robot đi thực tế bằng cách tích lũy khoảng cách từ topic `/odom`

2. Dùng `ros2 bag record` để ghi dữ liệu mỗi lần chạy:
   ```bash
   ros2 bag record /odom /plan /navigate_to_pose/_action/feedback -o trial_navfn_1
   ```

3. Lập bảng so sánh kết quả thử nghiệm:

| Planner | Lần chạy | Thời gian (s) | Quãng đường (m) | Có recovery |
|---|---|---|---|---|
| NavFn | 1 | | | |
| NavFn | 2 | | | |
| SmacPlanner2D | 1 | | | |
| SmacPlanner2D | 2 | | | |

4. Tính trung bình và độ lệch chuẩn cho mỗi planner, đưa ra kết luận

**Gợi ý:** Để tính quãng đường từ `/odom`, subscribe và tích lũy `sqrt(dx*dx + dy*dy)` giữa các message kế tiếp nhau.

---

### Bài 3 -- Custom NAV2 Controller Plugin bằng C++ (Nâng cao)

**Đề bài:** Viết một **custom controller plugin** cho NAV2 bằng C++ theo kiến trúc `nav2_core::Controller`, thực hiện thuật toán Pure Pursuit đơn giản.

**Logic controller cần implement:**
- Nhận `path` từ global planner qua `setPlan()`
- Trong mỗi lần gọi `computeVelocityCommands()`, tìm điểm look-ahead trên path cách robot một khoảng `lookahead_dist`
- Tính `angular.z` để robot quay về hướng điểm look-ahead
- Đặt `linear.x` bằng một hằng số (ví dụ 0.15 m/s)
- Trả về vận tốc bằng 0 nếu robot đã cách đích dưới `goal_tolerance`

**Cấu trúc package:**

```
my_pure_pursuit/
├── CMakeLists.txt
├── package.xml
├── plugins.xml
├── include/my_pure_pursuit/
│   └── pure_pursuit_controller.hpp
└── src/
    └── pure_pursuit_controller.cpp
```

**Interface cần implement:**

```cpp
#include "nav2_core/controller.hpp"

class PurePursuitController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  nav_msgs::msg::Path m_current_plan;
  double m_lookahead_dist{0.5};
  double m_linear_vel{0.15};
  rclcpp::Logger m_logger{rclcpp::get_logger("PurePursuitController")};
};
```

**Đăng ký plugin trong `plugins.xml`:**

```xml
<library path="my_pure_pursuit">
  <class name="my_pure_pursuit/PurePursuitController"
         type="my_pure_pursuit::PurePursuitController"
         base_class_type="nav2_core::Controller">
    <description>Simple Pure Pursuit Controller</description>
  </class>
</library>
```

**Tiêu chí đánh giá:**
- Plugin biên dịch không lỗi, đăng ký vào `pluginlib` thành công
- NAV2 có thể nạp plugin (không crash, có log "PurePursuitController configured")
- TurtleBot3 đi được đến đích ít nhất trong môi trường thẳng
- Báo cáo giải thích công thức tính `angular.z` từ góc sai lệch hướng đến look-ahead point

---

## 5. Rubric Đánh Giá

| STT | Tiêu chí |
|---|---|
| Lab 1 | Báo cáo quan sát node/topic/action, có ảnh rqt_graph |
| Lab 2 | Bản đồ được tạo và lưu, giải thích đúng file PGM và YAML |
| Lab 3 | Phân tích costmap với 3 mức inflation_radius, có ảnh chụp so sánh |
| Lab 4 | Node C++ gửi goal chạy được, xử lý đủ 3 callback |
| Lab 5 | Đổi planner/controller thành công, có nhận xét so sánh |
| Lab 6 | Visualize BT đúng, chỉnh sửa XML và kiểm tra hành vi |
| Bài tập 1 | Chạy đúng, có xử lý ABORTED, có video |
| Bài tập 2 | Có bảng số liệu, có kết luận |
| Bài tập 3 | Biên dịch và chạy được |

---