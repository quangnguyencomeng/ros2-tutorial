#pragma once
#include <cmath>
#include <limits>   // std::numeric_limits

// https://www.mathworks.com/help/robotics/ref/ackermannkinematics.html#d127e186

class AckermannKinematics
{
public:
  AckermannKinematics(double wheelbase, double track_width_front)
  : L_(wheelbase), T_(track_width_front) {}

  // Turning radius từ steering angle trung tâm φ
  double turningRadius(double phi) const
  {
    if (std::abs(phi) < 1e-9)
      return std::numeric_limits<double>::infinity();
    return L_ / std::tan(phi);
  }

  // Bánh trong (trái khi φ > 0)
  double computeLeftSteeringAngle(double phi) const
  {
    double R = turningRadius(phi);
    if (std::isinf(R)) return 0.0;
    return std::atan(L_ / (R - T_ / 2.0));
  }

  // Bánh ngoài (phải khi φ > 0)
  double computeRightSteeringAngle(double phi) const
  {
    double R = turningRadius(phi);
    if (std::isinf(R)) return 0.0;
    return std::atan(L_ / (R + T_ / 2.0));
  }

  // Tính lại central angle từ hai bánh (inverse)
  double computeCentralAngle(double phi_left, double phi_right) const
  {
    return (phi_left + phi_right) / 2.0;  // xấp xỉ đơn giản
  }

  double wheelbase() const { return L_; }

private:
  double L_;  // wheelbase
  double T_;  // track width front
};


// Hàm khởi tạo (Constructor): Nhận vào 2 tham số vật lý của xe:
    // L_ (Wheelbase): Chiều dài cơ sở (Khoảng cách từ trục bánh trước đến trục bánh sau).
    // T_ (Track width): Khoảng cách giữa bánh trước bên trái và bánh trước bên phải.

// Hàm turningRadius(phi): Tính bán kính vòng cua (R) của trọng tâm xe dựa vào góc bẻ lái giả lập ở trung tâm (phi).
    // Nếu phi xấp xỉ 0 (xe đi thẳng), bán kính rẽ sẽ là vô cực (infinity).
    // Ngược lại, áp dụng công thức: R = L / tan(phi).

// Hàm computeLeftSteeringAngle & computeRightSteeringAngle: Tính góc bẻ lái thực tế cho từng bánh.
    // Khi phi > 0 (rẽ trái), tâm vòng cua nằm ở bên trái. Bánh trái trở thành bánh trong (gần tâm cua hơn), công thức trừ đi một nửa track width (R - T_ / 2.0) làm cho mẫu số nhỏ đi, dẫn đến góc bẻ lái trái lớn hơn.
    // Ngược lại, bánh phải là bánh ngoài, công thức cộng thêm một nửa track width (R + T_ / 2.0), góc bẻ phải sẽ nhỏ hơn.

// Hàm computeCentralAngle: Đây là hàm Inverse Kinematics. Nó lấy trung bình cộng góc bẻ lái của hai bánh để ước lượng lại góc bẻ trung tâm (phi). (Trong thực tế, đây là một phép xấp xỉ đơn giản nhưng rất phổ biến).