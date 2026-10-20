cmake_minimum_required(VERSION 3.8)
project(cpp_service_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_service_server src/server.cpp)
ament_target_dependencies(cpp_service_server rclcpp example_interfaces)

add_executable(cpp_service_client src/client.cpp)
ament_target_dependencies(cpp_service_client rclcpp example_interfaces)

install(TARGETS
  cpp_service_server
  cpp_service_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
