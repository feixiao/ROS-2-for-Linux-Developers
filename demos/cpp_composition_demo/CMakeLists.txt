cmake_minimum_required(VERSION 3.8)
project(cpp_composition_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# 1. 编译发布者组件动态库 (.so)
add_library(talker_component SHARED src/talker_component.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(talker_component "cpp_composition_demo::TalkerComponent")

# 2. 编译订阅者组件动态库 (.so)
add_library(listener_component SHARED src/listener_component.cpp)
ament_target_dependencies(listener_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(listener_component "cpp_composition_demo::ListenerComponent")

# 3. 安装动态库与 launch 脚本
install(TARGETS
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
