#include "sar_bt_mission/mark_target_served_action.hpp"

#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

MarkTargetServedAction::MarkTargetServedAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList MarkTargetServedAction::providedPorts()
{
  return {
    BT::InputPort<int>("target_id",
      "구조 완료 처리할 요구조자 ID"),
    BT::InputPort<std::vector<int>>("in_served_ids",
      "현재 served_target_ids 값 (읽기용)"),
    BT::OutputPort<std::vector<int>>("out_served_ids",
      "target_id 추가 후 served_target_ids 값 (쓰기용)"),
  };
}

BT::NodeStatus MarkTargetServedAction::tick()
{
  int target_id = -1;
  if (!getInput("target_id", target_id)) {
    RCLCPP_ERROR(rclcpp::get_logger("MarkTargetServed"),
      "target_id 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }

  // 기존 목록 읽기 (초기 상태에서 BB에 값이 없을 수 있으므로 실패 허용)
  std::vector<int> served_ids;
  if (!getInput("in_served_ids", served_ids)) {
    served_ids.clear();
  }

  // 중복 추가 방지 (안전망 — TargetConfirmed에서 이미 걸러지지만 방어적 처리)
  const bool already_exists =
    std::find(served_ids.begin(), served_ids.end(), target_id) != served_ids.end();

  if (!already_exists) {
    served_ids.push_back(target_id);
    RCLCPP_INFO(rclcpp::get_logger("MarkTargetServed"),
      "✓ ID %d 구조 완료 등록 (총 구조 인원: %zu명)",
      target_id, served_ids.size());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("MarkTargetServed"),
      "ID %d 는 이미 등록된 상태 (중복 호출 무시)", target_id);
  }

  // Blackboard에 갱신된 벡터 저장
  setOutput("out_served_ids", served_ids);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sar_bt_mission
