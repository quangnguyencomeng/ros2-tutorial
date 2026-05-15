cmake_minimum_required(VERSION 3.8)
project(patrol_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Đăng ký file thực thi
add_executable(patrol_node src/patrol_node.cpp)

# XÓA HOẶC COMMENT ĐOẠN target_link_libraries CŨ ĐI
# CHỈ DÙNG ĐOẠN DƯỚI NÀY LÀ ĐỦ:
ament_target_dependencies(patrol_node 
  rclcpp 
  rclcpp_action 
  nav2_msgs 
  geometry_msgs
  tf2 
  tf2_geometry_msgs
)

install(TARGETS
  patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()