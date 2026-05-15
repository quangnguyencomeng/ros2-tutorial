cmake_minimum_required(VERSION 3.8)
project(ackermann_lifecycle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ackermann_kinematics_node src/ackermann_kinematics_node.cpp)
ament_target_dependencies(ackermann_kinematics_node
  rclcpp rclcpp_lifecycle ackermann_msgs nav_msgs tf2_ros geometry_msgs lifecycle_msgs
)

add_executable(kinematics_compare_node src/kinematics_compare_node.cpp)
ament_target_dependencies(kinematics_compare_node
  rclcpp nav_msgs geometry_msgs
)

add_executable(ackermann_system_manager src/ackermann_system_manager.cpp)
ament_target_dependencies(ackermann_system_manager
  rclcpp lifecycle_msgs std_srvs std_msgs
)

add_executable(ackermann_noise_node src/ackermann_noise_node.cpp)
ament_target_dependencies(ackermann_noise_node rclcpp nav_msgs geometry_msgs)

install(TARGETS 
  kinematics_compare_node
  ackermann_kinematics_node
  ackermann_system_manager
  ackermann_noise_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
