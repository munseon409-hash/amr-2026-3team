/**
 * @file main.cpp
 * @brief SAR (수색·구조) 로봇 BT 미션 컨트롤러 메인 노드
 *
 * ── 실행 흐름 ──────────────────────────────────────────────────────────────
 *  1. ROS 2 노드 초기화 및 파라미터 선언
 *  2. BehaviorTreeFactory에 커스텀 노드 등록
 *  3. XML 트리 로드 및 Blackboard 초기값 주입
 *  4. 검출 결과 구독자 생성 → Blackboard 실시간 갱신
 *  5. 10Hz Wall Timer에서 tree.tickOnce() 호출
 *  6. rclcpp::spin() — Nav2 액션 콜백 및 LiDAR 콜백 처리
 *
 * ── Blackboard 변수 (외부에서 주입) ────────────────────────────────────────
 *  raw_detection        bool   /red_detected       (std_msgs/Bool)
 *  current_target_point Point  /red_target_point   (geometry_msgs/Point)
 *
 * ── 웨이포인트 ROS 파라미터 ────────────────────────────────────────────────
 *  wp1_x, wp1_y, wp1_yaw   (default: 2.0, 0.0, 0.0)
 *  wp2_x, wp2_y, wp2_yaw   (default: 2.0, 2.0, 1.5708)
 */

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS 메시지 타입
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// 커스텀 BT 노드 헤더
#include "sar_bt_mission/align_to_target_action.hpp"
#include "sar_bt_mission/navigate_to_target_action.hpp"
#include "sar_bt_mission/navigate_to_waypoint_action.hpp"
#include "sar_bt_mission/safe_distance_reached_condition.hpp"
#include "sar_bt_mission/scan_at_waypoint_action.hpp"
#include "sar_bt_mission/voice_interaction_action.hpp"
#include "sar_bt_mission/publish_rescue_report_action.hpp"
#include "sar_bt_mission/mark_target_served_action.hpp"

// ── 유틸: yaw → quaternion 변환 (tf2 없이 내부 계산) ─────────────────────
static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

static geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw,
  const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id    = frame_id;
  ps.pose.position.x    = x;
  ps.pose.position.y    = y;
  ps.pose.position.z    = 0.0;
  ps.pose.orientation   = yawToQuaternion(yaw);
  return ps;
}

// ────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ── 1. ROS 2 노드 생성 ───────────────────────────────────────────────────
  auto node = std::make_shared<rclcpp::Node>("sar_bt_node");

  // ── 웨이포인트 파라미터 선언 (launch 또는 param 파일로 오버라이드 가능) ─
  node->declare_parameter("wp1_x",   5.436);
  node->declare_parameter("wp1_y",  -1.228);
  node->declare_parameter("wp1_yaw", 0.0);
  node->declare_parameter("wp2_x",   3.607);
  node->declare_parameter("wp2_y",  -1.131);
  node->declare_parameter("wp2_yaw", 0.0);
  node->declare_parameter("bt_tick_hz", 10.0);

  const double wp1_x   = node->get_parameter("wp1_x").as_double();
  const double wp1_y   = node->get_parameter("wp1_y").as_double();
  const double wp1_yaw = node->get_parameter("wp1_yaw").as_double();
  const double wp2_x   = node->get_parameter("wp2_x").as_double();
  const double wp2_y   = node->get_parameter("wp2_y").as_double();
  const double wp2_yaw = node->get_parameter("wp2_yaw").as_double();
  const double tick_hz = node->get_parameter("bt_tick_hz").as_double();

  RCLCPP_INFO(node->get_logger(), "SAR BT 노드 시작");
  RCLCPP_INFO(node->get_logger(), "WP1: (%.2f, %.2f, yaw=%.3f)", wp1_x, wp1_y, wp1_yaw);
  RCLCPP_INFO(node->get_logger(), "WP2: (%.2f, %.2f, yaw=%.3f)", wp2_x, wp2_y, wp2_yaw);

  // ── 2. BT Factory 생성 및 노드 등록 ─────────────────────────────────────
  BT::BehaviorTreeFactory factory;

  // ── 2a. 커스텀 비동기 노드 (ROS 노드 필요) ──────────────────────────────
  factory.registerBuilder<sar_bt_mission::AlignToTargetAction>(
    "AlignToTarget",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<sar_bt_mission::AlignToTargetAction>(name, config, node);
    });

  // ── 2b. ROS 노드가 필요한 커스텀 노드 (람다 빌더 패턴) ──────────────────
  factory.registerBuilder<sar_bt_mission::SafeDistanceReachedCondition>(
    "SafeDistanceReached",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<sar_bt_mission::SafeDistanceReachedCondition>(name, config, node);
    });

  factory.registerBuilder<sar_bt_mission::ScanAtWaypointAction>(
    "ScanAtWaypoint",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<sar_bt_mission::ScanAtWaypointAction>(name, config, node);
    });

  factory.registerBuilder<sar_bt_mission::VoiceInteractionAction>(
    "VoiceInteraction",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<sar_bt_mission::VoiceInteractionAction>(name, config, node);
    });

  factory.registerBuilder<sar_bt_mission::PublishRescueReportAction>(
    "PublishRescueReport",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<sar_bt_mission::PublishRescueReportAction>(name, config, node);
    });

  // ── 2c. Nav2 RosActionNode 등록 ─────────────────────────────────────────
  BT::RosNodeParams ros_params;
  ros_params.nh                      = node;
  ros_params.default_port_value      = "navigate_to_pose";
  ros_params.server_timeout          = std::chrono::milliseconds(5000);
  ros_params.wait_for_server_timeout = std::chrono::seconds(5);

  // BT.ROS2 이 버전에서는 registerNodeType<T>(name, params) 패턴 사용
  factory.registerNodeType<sar_bt_mission::NavigateToTargetAction>(
    "NavigateToTarget", ros_params);
  factory.registerNodeType<sar_bt_mission::NavigateToWaypointAction>(
    "NavigateToWaypoint", ros_params);

  RCLCPP_INFO(node->get_logger(), "BT 노드 등록 완료");

  // ── 3. XML 트리 로드 ─────────────────────────────────────────────────────
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("sar_bt_mission");
  const std::string tree_path = pkg_share + "/behavior_trees/sar_mission.xml";

  RCLCPP_INFO(node->get_logger(), "트리 로드: %s", tree_path.c_str());

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(tree_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(),
      "트리 로드 실패: %s\n"
      "경로 확인: ros2 pkg prefix sar_bt_mission", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // ── 4. Blackboard 초기값 설정 ─────────────────────────────────────────────
  auto blackboard = tree.rootBlackboard();

  blackboard->set("raw_detection", false);

  geometry_msgs::msg::Point default_point;
  default_point.x = 0.0;
  default_point.y = 0.0;
  default_point.z = 0.0;
  blackboard->set("current_target_point", default_point);

  const auto default_pose = makePose(0.0, 0.0, 0.0);
  blackboard->set("locked_target_pose", default_pose);

  // 웨이포인트 주입
  blackboard->set("waypoint_1", makePose(wp1_x, wp1_y, wp1_yaw));
  blackboard->set("waypoint_2", makePose(wp2_x, wp2_y, wp2_yaw));

  RCLCPP_INFO(node->get_logger(), "Blackboard 초기화 완료");

  // ── 5. 검출 결과 구독자 생성 ─────────────────────────────────────────────
  //
  //  검출 노드가 아래 토픽으로 데이터를 발행해야 함:
  //    /red_detected       std_msgs/Bool
  //    /red_target_point   geometry_msgs/Point
  //
  auto sub_detected = node->create_subscription<std_msgs::msg::Bool>(
    "/red_detected", rclcpp::SensorDataQoS(),
    [blackboard](const std_msgs::msg::Bool::SharedPtr msg) {
      blackboard->set("raw_detection", msg->data);
    });

  auto sub_target_point = node->create_subscription<geometry_msgs::msg::Point>(
    "/red_target_point", rclcpp::SensorDataQoS(),
    [blackboard](const geometry_msgs::msg::Point::SharedPtr msg) {
      blackboard->set("current_target_point", *msg);
    });

  RCLCPP_INFO(node->get_logger(), "검출 구독자 생성 완료");

  // ── 5.5 bt_navigator active 상태 대기 ────────────────────────────────────
  {
    auto lc_client = node->create_client<lifecycle_msgs::srv::GetState>(
      "/bt_navigator/get_state");
    RCLCPP_INFO(node->get_logger(), "bt_navigator 활성화 대기 중...");
    constexpr uint8_t ACTIVE = 3;
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (!lc_client->wait_for_service(std::chrono::milliseconds(500))) {
        continue;
      }
      auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto fut = lc_client->async_send_request(req);
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (rclcpp::ok() &&
             fut.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready &&
             std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
      }
      if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready &&
          fut.get()->current_state.id == ACTIVE) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(node->get_logger(), "Nav2 준비 완료 — BT 시작");
  }

  // ── 5.7 AMCL 유효 pose 대기 (2D Pose Estimate 완료될 때까지) ───────────────
  {
    RCLCPP_INFO(node->get_logger(), "AMCL 초기 pose 대기 중... RViz에서 2D Pose Estimate를 찍어주세요.");
    bool pose_received = false;
    auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::SystemDefaultsQoS(),
      [&pose_received](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // covariance[0] (x 분산)이 1.0 미만이면 수렴된 pose로 판단
        if (msg->pose.covariance[0] < 1.0) {
          pose_received = true;
        }
      });
    while (rclcpp::ok() && !pose_received) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_INFO(node->get_logger(), "AMCL pose 수신 완료 — BT 시작");
  }

  // ── 6. BT Tick 타이머 설정 (10Hz) ────────────────────────────────────────
  //
  //  단일 스레드 Executor(rclcpp::spin) 내에서 Wall Timer 콜백으로 BT를 구동.
  //  Nav2 액션 콜백, LiDAR 구독 콜백, BT Tick이 모두 같은 스레드에서 직렬 실행.
  //  → 별도의 mutex 없이 Blackboard 접근 안전 (단, SafeDistanceReached 내
  //    scan_mutex_ 는 혹시 모를 MultiThreadedExecutor 전환을 대비한 보호임).
  //
  const auto tick_period = std::chrono::duration<double>(1.0 / tick_hz);
  const auto tick_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tick_period);

  auto bt_timer = node->create_wall_timer(
    tick_ms,
    [&tree, &node]() {
      const BT::NodeStatus status = tree.tickOnce();

      // 트리가 SUCCESS/FAILURE로 종료되면 로그만 출력하고 계속 진행.
      // (Patrol의 Repeat(-1)이 있으므로 정상 상황에서는 RUNNING 유지됨)
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO_ONCE(node->get_logger(),
          "BT 트리 SUCCESS — 다음 Tick에서 재시작");
      } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(node->get_logger(),
          "BT 트리 FAILURE — Blackboard 상태 확인 필요");
      }
    });

  RCLCPP_INFO(node->get_logger(),
    "BT Tick 타이머 시작 (%.0f Hz) — Ctrl+C로 종료", tick_hz);

  // ── 7. Executor 스핀 ─────────────────────────────────────────────────────
  rclcpp::spin(node);

  // ── 8. 정리 ──────────────────────────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "종료 신호 수신 — BT 트리 정리 중...");
  tree.haltTree();  // 모든 RUNNING 노드에 Halt 신호 전송 (Nav2 CancelGoal 포함)

  rclcpp::shutdown();
  return 0;
}
