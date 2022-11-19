cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_tracking_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)



add_executable(controller
src/controller.cpp)
target_include_directories(controller
PUBLIC include)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  Eigen3
  tf2_ros
  geometry_msgs
  trajectory_tracking_msgs
)


ament_target_dependencies(controller ${THIS_PACKAGE_INCLUDE_DEPENDS} )

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)



ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/controller_interface.py)


ament_package()
