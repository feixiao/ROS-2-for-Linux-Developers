cmake_minimum_required(VERSION 3.8)
project(cpp_behavior_tree_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_executive src/mission_executive_node.cpp)
ament_target_dependencies(mission_executive
  rclcpp
  std_msgs
)

install(TARGETS
  mission_executive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
