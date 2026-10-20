cmake_minimum_required(VERSION 3.8)
project(nav2_costmap_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config maps launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
