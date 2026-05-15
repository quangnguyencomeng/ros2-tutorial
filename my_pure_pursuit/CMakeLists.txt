cmake_minimum_required(VERSION 3.8)
project(my_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm tất cả dependencies TRƯỚC
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

# 2. Khai báo thư viện (Biên dịch code C++) - BẠN ĐANG THIẾU CÁI NÀY
add_library(${PROJECT_NAME} SHARED
  src/pure_pursuit_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  nav2_core
  rclcpp
  pluginlib
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_util
)

# 3. Xuất file plugin (Lệnh này phải nằm SAU find_package pluginlib)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# 4. Cài đặt (Install)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()