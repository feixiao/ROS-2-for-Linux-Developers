cmake_minimum_required(VERSION 3.8)
project(cpp_ros2_control_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 编译差速驱动硬件接口动态链接库 (.so)
add_library(diff_drive_hardware SHARED
  src/diff_drive_hardware.cpp
)

ament_target_dependencies(diff_drive_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 导出 pluginlib 描述描述文件
pluginlib_export_plugin_description_file(hardware_interface diff_drive_hardware.xml)

install(TARGETS diff_drive_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
