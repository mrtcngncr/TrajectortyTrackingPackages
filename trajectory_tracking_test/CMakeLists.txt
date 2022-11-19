cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_tracking_msgs REQUIRED)
find_package(trajectory_tracking REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

install(
  PROGRAMS
    scripts/data_saver.py
    scripts/controller_test.py
    
  DESTINATION
    lib/${PROJECT_NAME}/
)

ament_package()
