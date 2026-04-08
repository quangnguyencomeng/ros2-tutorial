cmake_minimum_required(VERSION 3.8)
project(distance_warning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

#do tự tạo action nên cần thêm pkg liên quan đến action vào để ROS2 biên dịch được action
find_package(rosidl_default_generators REQUIRED) 
find_package(action_msgs REQUIRED)


# bt2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)




# 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetThreshold.srv"
  "action/CheckDistance.action"
  DEPENDENCIES action_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# mỗi node (file cpp) đều phải khai báo để biên dịch thành file thực thi (executable)

#1. khai báo tạo executables
# câu lệnh thêm executable: add_executable(<node_name> <file_name.cpp>)
add_executable(distance_publisher src/distance_publisher.cpp)
add_executable(distance_listener src/distance_listener.cpp)
add_executable(set_threshold_service src/set_threshold_service.cpp)
add_executable(distance_action_server src/distance_action_server.cpp)
add_executable(distance_action_client src/distance_action_client.cpp)
add_executable(distance_tf_broadcaster src/distance_tf_broadcaster.cpp)
add_executable(distance_tf_listener src/distance_tf_listener.cpp)
add_executable(distance_publisher_qos src/distance_publisher_qos.cpp)
add_executable(distance_listener_qos src/distance_listener_qos.cpp)
#2. Dependencies và Linking
# Trình biên dịch cần biết node nào sử dụng thư viện nào (std_msgs, rclcpp,...)
# và node nào sử dụng cái Action CheckDistance mà mình tự tạo

# câu lệnh set thư viện cho mỗi node: ament_target_dependencies(<file_name> <lib_name1> <lib_name2> ...)
# các thư viện phổ biến:
  # rclcpp
  # std_msgs
  # std_srvs 
  # rclcpp_action
  # action_msgs
ament_target_dependencies(distance_publisher rclcpp std_msgs)
ament_target_dependencies(distance_listener rclcpp std_msgs)
ament_target_dependencies(distance_tf_broadcaster rclcpp std_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(distance_tf_listener rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(distance_publisher_qos rclcpp std_msgs)
ament_target_dependencies(distance_listener_qos rclcpp std_msgs tf2_ros)

target_link_libraries(distance_tf_broadcaster "${cpp_typesupport_target}")
target_link_libraries(distance_tf_broadcaster "${cpp_typesupport_target}")


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_target_dependencies(set_threshold_service rclcpp std_srvs)
add_dependencies(set_threshold_service ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(set_threshold_service "${cpp_typesupport_target}")
# 2 file action_... sử dụng thư viện action_msgs
ament_target_dependencies(distance_action_server rclcpp rclcpp_action action_msgs) 
ament_target_dependencies(distance_action_client rclcpp rclcpp_action action_msgs)
ament_export_dependencies(rosidl_default_runtime)

# liên kết Action tự định nghĩa vào các node Action


# link target đó vào 2 node xử lý Action (để chúng hiểu cấu trúc msg)
target_link_libraries(distance_action_server "${cpp_typesupport_target}")
target_link_libraries(distance_action_client "${cpp_typesupport_target}")

target_link_libraries(distance_tf_listener "${cpp_typesupport_target}")

# 3. Cài đặt các file thực thi để có thể dùng lệnh "ros2 run"

install( TARGETS
  distance_tf_broadcaster
  distance_tf_listener
  distance_action_client
  distance_action_server
  distance_listener
  distance_publisher
  set_threshold_service
  distance_publisher_qos 
  distance_listener_qos

  DESTINATION lib/${PROJECT_NAME}
) 

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
ament_package() # luôn là dòng cuối


