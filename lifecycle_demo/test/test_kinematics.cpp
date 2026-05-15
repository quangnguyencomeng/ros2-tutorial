#include <gtest/gtest.h>
#include <cmath>
#include "lifecycle_demo/ackermann_kinematics.hpp"

class KinematicsTest : public ::testing::Test {
protected:
  AckermannKinematics kin{1.5, 1.0};  // wheelbase=1.5m, track=1.0m
};

TEST_F(KinematicsTest, ZeroAngle) {
  EXPECT_DOUBLE_EQ(kin.computeLeftSteeringAngle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kin.computeRightSteeringAngle(0.0), 0.0);
  EXPECT_TRUE(std::isinf(kin.turningRadius(0.0)));
}

TEST_F(KinematicsTest, PositiveAngle) {
  double phi = 0.3;  // rad
  double left  = kin.computeLeftSteeringAngle(phi);
  double right = kin.computeRightSteeringAngle(phi);
  // Bánh trong (trái) phải lớn hơn bánh ngoài (phải)
  EXPECT_GT(left, right);
  EXPECT_GT(left, 0.0);
  EXPECT_GT(right, 0.0);
}

TEST_F(KinematicsTest, Symmetry) {
  double phi = 0.3;
  // Đổi chiều thì đổi dấu, magnitude như nhau
  EXPECT_NEAR(kin.computeLeftSteeringAngle(phi),
              -kin.computeRightSteeringAngle(-phi), 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


/*
class KinematicsTest (Test Fixture): Khởi tạo một chiếc xe ảo (kin) dùng chung cho mọi bài test với wheelbase = 1.5m và track_width = 1.0m.

TEST_F(KinematicsTest, ZeroAngle): Kiểm tra trường hợp xe đi thẳng (phi = 0).
    EXPECT_DOUBLE_EQ: Kỳ vọng góc bánh trái và phải đều chính xác bằng 0.0.
    EXPECT_TRUE: Kỳ vọng hàm tính bán kính vòng cua phải trả về kết quả là vô cực (std::isinf).

TEST_F(KinematicsTest, PositiveAngle): Kiểm tra trường hợp xe rẽ trái (phi = 0.3 > 0).
    EXPECT_GT(left, right): Khẳng định tính chất quan trọng nhất của Ackermann: Bánh trong (trái) phải lớn hơn bánh ngoài (phải).
    Góc bẻ của cả hai bánh đều phải lớn hơn 0.0 (cùng bẻ về bên trái).

TEST_F(KinematicsTest, Symmetry): Kiểm tra tính đối xứng của xe.
    Nếu bạn bẻ lái sang trái một góc 0.3, góc của bánh bên trái sẽ phải bằng chính xác góc của bánh bên phải khi bạn bẻ lái sang phải một góc -0.3.
    EXPECT_NEAR: Dùng để so sánh hai số thực (float/double) có bằng nhau hay không với một sai số cho phép là 1e-9 (vì tính toán lượng giác atan2 và tan thường sinh ra sai số làm tròn rất nhỏ trên máy tính).

Hàm main: Gọi RUN_ALL_TESTS() để kích hoạt thư viện Google Test quét và chạy toàn bộ các TEST_F bên trên. Tốc độ kiểm tra tính bằng mili-giây.*/