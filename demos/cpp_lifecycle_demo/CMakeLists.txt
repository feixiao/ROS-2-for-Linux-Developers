cmake_minimum_required(VERSION 3.8)
project(cpp_lifecycle_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lifecycle_talker src/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

install(TARGETS
  lifecycle_talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
