cmake_minimum_required(VERSION 3.8)
project(lifecycle_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)



# Khai báo file thực thi với tên là minimal_lifecycle_node
add_executable(minimal_lifecycle_node src/minimal_lifecycle_node.cpp)

# Link các thư viện cần thiết của ROS2
ament_target_dependencies(minimal_lifecycle_node
  rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# Install file thực thi vào thư mục lib
install(TARGETS 
  minimal_lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(minimal_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# --- THÊM PHẦN CẤU HÌNH TEST ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Tạo một executable test có tên là test_kinematics
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  
  # Chỉ định thư mục include cho file test
  target_include_directories(test_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()



ament_package()