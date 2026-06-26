cmake_minimum_required(VERSION 3.8)
project(sar_bt_mission)

# ── 컴파일러 경고 설정 ────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 의존성 패키지 ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ROS 메시지/서비스 타입
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ── 소스 파일 목록 ────────────────────────────────────────────────────────
set(SAR_BT_SOURCES
  src/target_confirmed_condition.cpp
  src/lock_target_action.cpp
  src/align_to_target_action.cpp
  src/safe_distance_reached_condition.cpp
  src/scan_at_waypoint_action.cpp
  src/voice_interaction_action.cpp
  src/publish_rescue_report_action.cpp
  src/mark_target_served_action.cpp
  src/main.cpp
)

# ── 실행 파일 빌드 ────────────────────────────────────────────────────────
add_executable(sar_bt_node ${SAR_BT_SOURCES})

target_include_directories(sar_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(sar_bt_node PUBLIC cxx_std_17)

ament_target_dependencies(sar_bt_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  ament_index_cpp
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2
  lifecycle_msgs
)

# ── 설치 규칙 ─────────────────────────────────────────────────────────────
install(TARGETS sar_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# XML 트리 파일을 share 디렉토리에 설치 (ament_index_cpp로 경로 탐색)
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/red_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── 테스트 (선택) ─────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
