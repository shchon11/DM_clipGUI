cmake_minimum_required(VERSION 3.8)
project(clip_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(clip_recorder src/clip_recorder.cpp)
target_compile_features(clip_recorder PUBLIC cxx_std_17)
ament_target_dependencies(clip_recorder
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  std_msgs
  std_srvs
  diagnostic_msgs
)

install(TARGETS clip_recorder
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/buffer_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buffer_probe)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
