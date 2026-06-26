#include "sar_bt_mission/scan_at_waypoint_action.hpp"

#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

ScanAtWaypointAction::ScanAtWaypointAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS());
}

BT::PortsList ScanAtWaypointAction::providedPorts()
{
  return {};  // 현재 포트 없음 — 필요 시 angular_vel, hold_duration 추가 가능
}

// ── 내부 헬퍼 ──────────────────────────────────────────────────────────────

void ScanAtWaypointAction::publishTwist(double angular_z)
{
  geometry_msgs::msg::Twist twist;
  twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist);
}

void ScanAtWaypointAction::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

void ScanAtWaypointAction::transitionTo(ScanPhase next_phase)
{
  phase_       = next_phase;
  phase_start_ = node_->now();

  switch (next_phase) {
    case ScanPhase::ROTATE_LEFT:
      RCLCPP_INFO(node_->get_logger(),
        "[%s] 스캔 Phase 1: 좌측 40° 회전 (%.1fs)", name().c_str(), DURATION_40);
      publishTwist(+ANGULAR_VEL_RAD_S);
      break;
    case ScanPhase::ROTATE_RIGHT:
      RCLCPP_INFO(node_->get_logger(),
        "[%s] 스캔 Phase 2: 우측 80° 회전 (%.1fs)", name().c_str(), DURATION_80);
      publishTwist(-ANGULAR_VEL_RAD_S);
      break;
    case ScanPhase::RECENTER:
      RCLCPP_INFO(node_->get_logger(),
        "[%s] 스캔 Phase 3: 정면 복귀 좌측 40° (%.1fs)", name().c_str(), DURATION_40);
      publishTwist(+ANGULAR_VEL_RAD_S);
      break;
    case ScanPhase::HOLD:
      RCLCPP_INFO(node_->get_logger(),
        "[%s] 스캔 Phase 4: 정지 홀드 (%.1fs)", name().c_str(), HOLD_DURATION_S);
      publishStop();
      break;
    case ScanPhase::DONE:
      publishStop();
      break;
  }
}

// ── StatefulActionNode 인터페이스 ──────────────────────────────────────────

BT::NodeStatus ScanAtWaypointAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] 웨이포인트 스캔 시작", name().c_str());
  transitionTo(ScanPhase::ROTATE_LEFT);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ScanAtWaypointAction::onRunning()
{
  const double elapsed = (node_->now() - phase_start_).seconds();

  switch (phase_) {
    case ScanPhase::ROTATE_LEFT:
      if (elapsed >= DURATION_40) { transitionTo(ScanPhase::ROTATE_RIGHT); }
      break;

    case ScanPhase::ROTATE_RIGHT:
      if (elapsed >= DURATION_80) { transitionTo(ScanPhase::RECENTER); }
      break;

    case ScanPhase::RECENTER:
      if (elapsed >= DURATION_40) { transitionTo(ScanPhase::HOLD); }
      break;

    case ScanPhase::HOLD:
      if (elapsed >= HOLD_DURATION_S) {
        transitionTo(ScanPhase::DONE);
        RCLCPP_INFO(node_->get_logger(), "[%s] 스캔 완료", name().c_str());
        return BT::NodeStatus::SUCCESS;
      }
      break;

    case ScanPhase::DONE:
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ScanAtWaypointAction::onHalted()
{
  // Emergency 브랜치로 전환될 때 로봇을 즉시 정지시킴
  publishStop();
  RCLCPP_WARN(node_->get_logger(),
    "[%s] 스캔 중단 — 긴급 구조 브랜치로 전환", name().c_str());
}

}  // namespace sar_bt_mission
