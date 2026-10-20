cmake_minimum_required(VERSION 3.8)
project(multimedia_av_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# 1. 编译基于 OpenCV / image_transport 的通用相机发布节点
add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# 2. 编译基于 GStreamer 硬编加速的低延迟 H.264 视频流推流节点
add_executable(gst_h264_streamer src/gst_h264_streamer.cpp)
ament_target_dependencies(gst_h264_streamer
  rclcpp
  sensor_msgs
)

install(TARGETS
  camera_publisher
  gst_h264_streamer
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/audio_capture_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
