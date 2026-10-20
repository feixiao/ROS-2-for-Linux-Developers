cmake_minimum_required(VERSION 3.8)
project(cpp_tf2_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1. 动态坐标变换发布器
add_executable(dynamic_broadcaster src/dynamic_broadcaster.cpp)
ament_target_dependencies(dynamic_broadcaster rclcpp geometry_msgs tf2 tf2_ros)

# 2. 静态坐标变换发布器
add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster rclcpp geometry_msgs tf2 tf2_ros)

# 3. 坐标监听与查询器
add_executable(pose_listener src/pose_listener.cpp)
ament_target_dependencies(pose_listener rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  dynamic_broadcaster
  static_broadcaster
  pose_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
