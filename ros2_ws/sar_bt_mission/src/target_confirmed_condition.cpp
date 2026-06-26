#include "sar_bt_mission/target_confirmed_condition.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

TargetConfirmedCondition::TargetConfirmedCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList TargetConfirmedCondition::providedPorts()
{
  return {
    BT::InputPort<float>("confidence",
      "YOLO 검출 신뢰도 (0.0 ~ 1.0)"),
    BT::InputPort<float>("bbox_area",
      "바운딩 박스 면적 (픽셀²) — 근접 여부 판단"),
    BT::InputPort<int>("target_id",
      "실시간 검출된 요구조자 고유 ID"),
    BT::InputPort<std::vector<int>>("served_ids",
      "이미 구조 완료된 ID 목록"),
  };
}

BT::NodeStatus TargetConfirmedCondition::tick()
{
  float confidence = 0.0f;
  float bbox_area  = 0.0f;
  int   target_id  = -1;
  std::vector<int> served_ids;

  // 필수 포트 읽기 실패 시 FAILURE
  if (!getInput("confidence", confidence)) {
    RCLCPP_WARN(rclcpp::get_logger("TargetConfirmed"), "confidence 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("bbox_area", bbox_area)) {
    RCLCPP_WARN(rclcpp::get_logger("TargetConfirmed"), "bbox_area 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("target_id", target_id)) {
    RCLCPP_WARN(rclcpp::get_logger("TargetConfirmed"), "target_id 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }
  // served_ids는 초기엔 비어있을 수 있으므로 읽기 실패를 허용
  if (!getInput("served_ids", served_ids)) {
    served_ids.clear();
  }

  // ── 조건 1: 신뢰도 임계값 ──────────────────────────────
  if (confidence <= CONFIDENCE_THRESHOLD) {
    RCLCPP_DEBUG(rclcpp::get_logger("TargetConfirmed"),
      "신뢰도 부족: %.2f <= %.2f", confidence, CONFIDENCE_THRESHOLD);
    return BT::NodeStatus::FAILURE;
  }

  // ── 조건 2: BBox 면적 임계값 (너무 멀면 FAILURE) ────────
  if (bbox_area <= BBOX_AREA_THRESHOLD) {
    RCLCPP_DEBUG(rclcpp::get_logger("TargetConfirmed"),
      "BBox 면적 부족: %.1f <= %.1f px²", bbox_area, BBOX_AREA_THRESHOLD);
    return BT::NodeStatus::FAILURE;
  }

  // ── 조건 3: 중복 구조 방지 ──────────────────────────────
  if (std::find(served_ids.begin(), served_ids.end(), target_id) != served_ids.end()) {
    RCLCPP_INFO(rclcpp::get_logger("TargetConfirmed"),
      "ID %d 는 이미 구조 완료 — 건너뜀", target_id);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("TargetConfirmed"),
    "✓ 유효 대상 확인 — ID:%d, conf:%.2f, area:%.1f",
    target_id, confidence, bbox_area);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sar_bt_mission
