cmake_minimum_required(VERSION 3.8)
project(cpp_topic_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cpp_talker src/talker.cpp)
ament_target_dependencies(cpp_talker rclcpp std_msgs)

add_executable(cpp_listener src/listener.cpp)
ament_target_dependencies(cpp_listener rclcpp std_msgs)

install(TARGETS
  cpp_talker
  cpp_listener
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_talker test/test_talker.cpp)
  ament_target_dependencies(test_talker rclcpp std_msgs)
endif()

ament_package()
