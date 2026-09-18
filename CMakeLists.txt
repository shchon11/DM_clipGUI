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

# 송신 IP별 수신 트래픽 카운터 (AF_PACKET). 설치 후 한 번:
#   sudo setcap cap_net_raw+ep $(ros2 pkg prefix clip_recorder)/lib/clip_recorder/net_probe
add_executable(net_probe src/net_probe.c)
install(TARGETS net_probe
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/buffer_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buffer_probe)

install(PROGRAMS scripts/clip_trigger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME clip_trigger)

install(PROGRAMS scripts/clip_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME clip_gui)

install(PROGRAMS scripts/clip_launch.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME clip_launch)

# clip_gui가 import하는 모듈 — 실행 파일이 아니라 .py 확장자를 유지해야 함
install(FILES scripts/bag_diagnostics.py scripts/net_tools.py scripts/gnss_tools.py
  scripts/map_widget.py
  scripts/sensor_discovery.py scripts/sensor_config.py scripts/sensor_launcher.py
  scripts/sensor_stage.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
