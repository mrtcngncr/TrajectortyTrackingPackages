cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/data_saver.py

    
  DESTINATION
    lib/${PROJECT_NAME}/
)

ament_package()
