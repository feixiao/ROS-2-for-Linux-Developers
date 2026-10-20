cmake_minimum_required(VERSION 3.8)
project(cpp_action_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_action_server src/action_server.cpp)
ament_target_dependencies(cpp_action_server
  rclcpp
  rclcpp_action
  example_interfaces
)

add_executable(cpp_action_client src/action_client.cpp)
ament_target_dependencies(cpp_action_client
  rclcpp
  rclcpp_action
  example_interfaces
)

install(TARGETS
  cpp_action_server
  cpp_action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
