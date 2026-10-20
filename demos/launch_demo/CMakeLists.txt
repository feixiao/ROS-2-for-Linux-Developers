cmake_minimum_required(VERSION 3.8)
project(launch_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pipeline_integration.py)
endif()
ament_package()
