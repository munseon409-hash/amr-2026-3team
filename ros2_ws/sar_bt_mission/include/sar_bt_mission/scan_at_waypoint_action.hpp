#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sar_bt_mission {

/**
 * @brief 웨이포인트 도착 후 제자리 스캔을 수행하는 StatefulActionNode
 *
 * 스캔 시퀀스 (총 ~10.6초):
 *   Phase 1 ROTATE_LEFT : 좌측으로 40° 회전 (~1.4초, angular_z = +0.5 rad/s)
 *   Phase 2 ROTATE_RIGHT: 우측으로 80° 회전 (~2.8초, angular_z = -0.5 rad/s)
 *                          (좌 40° + 우 40° = 중심에서 우측 40°까지)
 *   Phase 3 RECENTER    : 좌측으로 40° 회전 (~1.4초) → 정면 복귀
 *   Phase 4 HOLD        : 정지 5초 (요구조자 감지 대기)
 *   → SUCCESS
 *
 * Halt 시 즉시 cmd_vel=0을 발행하여 로봇을 정지시킨다.
 */
class ScanAtWaypointAction : public BT::StatefulActionNode
{
public:
  ScanAtWaypointAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ── 스캔 파라미터 ──────────────────────────────────────
  static constexpr double ANGULAR_VEL_RAD_S = 0.5;   // 회전 속도 (rad/s)
  static constexpr double HOLD_DURATION_S   = 5.0;   // 홀드 시간 (초)

  // 40° / 80° → 라디안 → 소요 시간 계산
  static constexpr double DEG_40_RAD  = 40.0 * M_PI / 180.0;
  static constexpr double DEG_80_RAD  = 80.0 * M_PI / 180.0;
  static constexpr double DURATION_40 = DEG_40_RAD / ANGULAR_VEL_RAD_S;  // ~1.396s
  static constexpr double DURATION_80 = DEG_80_RAD / ANGULAR_VEL_RAD_S;  // ~2.793s

  enum class ScanPhase { ROTATE_LEFT, ROTATE_RIGHT, RECENTER, HOLD, DONE };

  void publishTwist(double angular_z);
  void publishStop();
  void transitionTo(ScanPhase next_phase);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  ScanPhase phase_{ScanPhase::ROTATE_LEFT};
  rclcpp::Time phase_start_;
};

}  // namespace sar_bt_mission
