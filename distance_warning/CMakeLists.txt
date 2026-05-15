cmake_minimum_required(VERSION 3.8)
project(distance_warning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # <-- Added rclcpp_action
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Custom Messages, Services, Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CheckDistance.action"
  "srv/SetThreshold.srv"
  DEPENDENCIES geometry_msgs
)
set(msg_srv_action_target "${PROJECT_NAME}__rosidl_typesupport_cpp")

# --- Executables ---

# Get the typesupport target once to use for all nodes that need custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 1. Service Node
add_executable(set_threshold_service src/set_threshold_service.cpp)
ament_target_dependencies(set_threshold_service rclcpp)
target_link_libraries(set_threshold_service "${cpp_typesupport_target}") # <-- Fixed deprecation

# 2. TF2 Listener (Lưu ý: node này dùng service nên phải đợi interface)
add_executable(distance_tf_listener src/distance_tf_listener.cpp)
ament_target_dependencies(distance_tf_listener rclcpp tf2 tf2_ros geometry_msgs)
target_link_libraries(distance_tf_listener "${cpp_typesupport_target}") # <-- Fixed deprecation

# 3. Action Nodes
add_executable(distance_action_server src/distance_action_server.cpp)
ament_target_dependencies(distance_action_server rclcpp rclcpp_action) # <-- Added rclcpp_action
target_link_libraries(distance_action_server "${cpp_typesupport_target}") # <-- Fixed deprecation

add_executable(distance_action_client src/distance_action_client.cpp)
ament_target_dependencies(distance_action_client rclcpp rclcpp_action) # <-- Added rclcpp_action
target_link_libraries(distance_action_client "${cpp_typesupport_target}") # <-- Fixed deprecation

# 4. Các node cơ bản khác
add_executable(distance_publisher src/distance_publisher.cpp)
ament_target_dependencies(distance_publisher rclcpp geometry_msgs sensor_msgs)

add_executable(distance_listener src/distance_listener.cpp)
ament_target_dependencies(distance_listener rclcpp geometry_msgs sensor_msgs)

add_executable(distance_publisher_qos src/distance_publisher_qos.cpp)
ament_target_dependencies(distance_publisher_qos rclcpp geometry_msgs sensor_msgs)

add_executable(distance_listener_qos src/distance_listener_qos.cpp)
ament_target_dependencies(distance_listener_qos rclcpp geometry_msgs sensor_msgs)

add_executable(distance_tf_broadcaster src/distance_tf_broadcaster.cpp)
ament_target_dependencies(distance_tf_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

# Add dependencies to ensure interfaces are built first
add_dependencies(set_threshold_service ${msg_srv_action_target})
add_dependencies(distance_tf_listener ${msg_srv_action_target})
add_dependencies(distance_action_server ${msg_srv_action_target})
add_dependencies(distance_action_client ${msg_srv_action_target})

# --- Install ---

install(TARGETS
  distance_publisher distance_listener
  distance_publisher_qos distance_listener_qos
  distance_tf_broadcaster distance_tf_listener
  set_threshold_service
  distance_action_server distance_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch maps action srv
  DESTINATION share/${PROJECT_NAME}
)

# Cài đặt file .yaml và .md trực tiếp ở thư mục gốc nếu cần
install(FILES my_nav2_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()