# Hướng Dẫn Lifecycle Node & Ackermann Steering trong ROS2

**Môi trường thực hành:** ROS2 Humble + Gazebo Simulation  
**Ngôn ngữ lập trình chính:** C++

---

# 1. Kiến Thức Cần Tìm Hiểu

## 1.1 Lifecycle Node trong ROS2

- Lifecycle Node là gì và tại sao cần dùng trong robot system
- Sự khác nhau giữa `rclcpp::Node` và `rclcpp_lifecycle::LifecycleNode`
- State machine của Lifecycle Node:
  - `Unconfigured`
  - `Inactive`
  - `Active`
  - `Finalized`
- Các transition:
  - `configure()`
  - `activate()`
  - `deactivate()`
  - `cleanup()`
  - `shutdown()`
- Lifecycle Publisher và activate/deactivate publisher

---

## 1.2 Ackermann Steering Kinematics

- Ackermann steering là gì
- Bicycle model
- Steering angle trung tâm `φ`
- Turning radius
- Wheelbase và track width
- Steering angle bánh trái và bánh phải
- Yaw rate `ω`

### Công thức turning radius

```math
R_b = \frac{l}{\tan(\phi)}
```

### Forward kinematics

```math
\omega = \frac{v}{l}\tan(\phi)
```

```math
\dot{x}=v\cos(\theta)
```

```math
\dot{y}=v\sin(\theta)
```

```math
\dot{\theta}=\omega
```

---

## 1.3 Ackermann Odometry

- Tích phân odometry
- Euler integration
- Odometry drift
- Steering encoder
- Velocity encoder
- Noise trong odometry

---

## 1.4 TF2 và Coordinate Frames

- `odom -> base_link`
- Ý nghĩa frame `odom`
- Ý nghĩa frame `base_link`
- Publish TF bằng `tf2_ros`
- Visualize bằng RViz2

---

## 1.5 ROS2 Interfaces

### Topics

- `/cmd_ackermann`
- `/wheel_commands`
- `/odom`
- `/path`

### Messages

- `ackermann_msgs/msg/AckermannDrive`
- `nav_msgs/msg/Odometry`
- `nav_msgs/msg/Path`

---

# 2. Tài Liệu Tham Khảo

## Tài liệu chính thức

| Tài liệu | Liên kết |
|---|---|
| ROS2 Lifecycle Design | https://design.ros2.org/articles/node_lifecycle.html |
| ROS2 Lifecycle API | https://docs.ros2.org/humble/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html |
| ros2_controllers Kinematics | https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html |
| ROS2 TF2 Tutorials | https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html |

---

# 3. Bài Luyện Tập / Tìm Hiểu Rõ Hơn

---

## Lab 1 -- Hello Lifecycle Node

**Mục tiêu:** Viết Lifecycle Node đầu tiên và quan sát state machine bằng CLI.

### Yêu cầu

1. Tạo package:

```bash
ros2 pkg create lifecycle_demo --build-type ament_cmake
```

2. Viết class:

```cpp
MinimalLifecycleNode
```

kế thừa:

```cpp
rclcpp_lifecycle::LifecycleNode
```

3. Override callback:
- `on_configure()`
- `on_activate()`
- `on_deactivate()`
- `on_cleanup()`
- `on_shutdown()`

4. Tạo lifecycle publisher `/status`

5. Publish message mỗi 1Hz khi node ở state Active

---

### Kiểm tra bằng CLI

```bash
ros2 lifecycle nodes

ros2 lifecycle get /minimal_lifecycle_node

ros2 lifecycle set /minimal_lifecycle_node configure

ros2 lifecycle set /minimal_lifecycle_node activate

ros2 topic echo /status
```

---

### Câu hỏi thảo luận

- Nếu `on_configure()` trả về FAILURE thì điều gì xảy ra?
- Lifecycle publisher có publish được khi node ở state Inactive không?
- Subscriber callback có nên xử lý dữ liệu khi node inactive không?

---

## Lab 2 -- Ackermann Kinematics Calculator

**Mục tiêu:** Implement Ackermann steering kinematics bằng C++.

### Yêu cầu

1. Tạo class:

```cpp
class AckermannKinematics
{
public:
  AckermannKinematics(
      double wheelbase,
      double track_width_front);

  double computeLeftSteeringAngle(double phi) const;

  double computeRightSteeringAngle(double phi) const;

  double computeCentralAngle(
      double phi_left,
      double phi_right) const;

  double turningRadius(double phi) const;
};
```

2. Implement:
- Turning radius
- Inverse kinematics
- Steering reconstruction

3. Viết unit test:
- `phi = 0`
- `phi > 0`
- symmetry test

---

### Kiểm tra

```text
phi = 0
→ phi_left = phi_right = 0
→ turning radius = infinity
```

```text
phi > 0
→ phi_left > phi_right
```

---

## Lab 3 -- Ackermann Lifecycle Node

**Mục tiêu:** Kết hợp Lifecycle Node và Ackermann kinematics.

### Yêu cầu

1. Tạo node:

```cpp
AckermannKinematicsNode
```

2. Trong `on_configure()`:
- đọc parameter:
  - `wheelbase`
  - `track_width_front`
- tạo publisher/subscriber/timer
- tạo TF broadcaster

3. Subscribe:

```text
/cmd_ackermann
```

Kiểu:

```text
ackermann_msgs/msg/AckermannDrive
```

4. Publish:
- `/wheel_commands`
- `/odom`
- `/path`

5. Publish TF:

```text
odom -> base_link
```

---

### Kiểm tra

```bash
ros2 topic pub /cmd_ackermann \
ackermann_msgs/msg/AckermannDrive \
"{speed: 1.0, steering_angle: 0.3}"
```

```bash
ros2 topic echo /wheel_commands
```

```bash
ros2 topic echo /odom
```

```bash
rviz2
```

---

### Câu hỏi thảo luận

- Tại sao `AckermannDrive` phù hợp hơn `geometry_msgs/Twist`?
- Steering angle `φ` khác gì yaw rate `ω`?
- Điều gì xảy ra khi `φ → ±90°`?

---

## Lab 4 -- Visualize Path và Odometry

**Mục tiêu:** Visualize quỹ đạo robot trong RViz2.

### Yêu cầu

1. Publish:

```text
nav_msgs/msg/Path
```

2. Visualize:
- Path
- TF
- Odometry

3. Chạy thử:
- vòng tròn
- chữ S
- đảo chiều

---

### Câu hỏi thảo luận

- Odometry drift xảy ra như thế nào?
- Nếu dùng:
```text
phi_left = phi_right
```

thì quỹ đạo có còn đúng Ackermann không?

---

# 4. Bài Tập Về Nhà

---

## Bài 1 -- Lifecycle Manager

**Đề bài:** Viết node quản lý lifecycle cho nhiều node.

### Yêu cầu

1. Node:

```text
AckermannSystemManager
```

2. Service:
- `/system/start`
- `/system/stop`

3. Start sequence:
```text
configure → activate
```

4. Stop sequence:
```text
deactivate → cleanup
```

5. Nếu node fail:
- rollback
- cleanup các node đã activate

6. Publish:
```text
/system/status
```

---

## Bài 2 -- Ackermann vs Differential Drive

### Yêu cầu

1. Implement:
- `DiffDriveKinematics`
- `AckermannKinematics`

2. Simulate 10 giây

3. Export CSV:

```text
time,
x_diff,
y_diff,
theta_diff,
x_ackermann,
y_ackermann,
theta_ackermann
```

4. Vẽ graph bằng matplotlib

---

### Câu hỏi

- Quỹ đạo có giống nhau không?
- Ackermann có constraint gì?
- Steering angle thực tế thường giới hạn bao nhiêu?

---

## Bài 3 -- Ackermann Odometry với Noise

### Yêu cầu

1. Add Gaussian noise:
- steering encoder
- velocity encoder

2. Compare:
- ground truth
- odometry

3. Test nhiều mức noise:
- `0.001`
- `0.01`
- `0.05`

---

# 5. Gợi Ý Cấu Trúc Package

```text
ackermann_lifecycle_pkg/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
├── test/
├── launch/
└── config/
```

---

# 6. Dependencies

```xml
rclcpp
rclcpp_lifecycle
lifecycle_msgs
ackermann_msgs
nav_msgs
tf2_ros
```

---

# 7. Yêu Cầu Nộp Bài

- Source code
- README
- Unit test result
- Screenshot RViz2
- Video demo
- Báo cáo PDF
