cmake_minimum_required(VERSION 3.8)
project(nav2_goal_sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(goal_sender src/goal_sender.cpp)
ament_target_dependencies(goal_sender 
  rclcpp 
  rclcpp_action 
  nav2_msgs
  tf2 
  tf2_geometry_msgs
)

# --- KHAI BÁO CHO PLANNER_COMPARATOR ---
add_executable(planner_comparator src/planner_comparison_node.cpp)
ament_target_dependencies(planner_comparator
  rclcpp 
  rclcpp_action 
  nav2_msgs
  nav_msgs        # Nhớ thêm nav_msgs vì code so sánh dùng Odometry
  geometry_msgs
)

# 3. Cài đặt các file thực thi vào hệ thống
install(TARGETS
  goal_sender
  planner_comparator
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
