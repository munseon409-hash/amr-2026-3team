#include "sar_bt_mission/lock_target_action.hpp"

#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

LockTargetAction::LockTargetAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList LockTargetAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>("input_point",
      "검출 노드로부터 실시간 갱신되는 current_target_point"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose",
      "주행 중 고정될 locked_target_pose (Nav2용 PoseStamped)"),
  };
}

BT::NodeStatus LockTargetAction::tick()
{
  geometry_msgs::msg::Point point;
  if (!getInput("input_point", point)) {
    RCLCPP_ERROR(rclcpp::get_logger("LockTarget"),
      "input_point(current_target_point) 읽기 실패 — Blackboard 미설정 확인");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = "map";
  pose.pose.position.x    = point.x;
  pose.pose.position.y    = point.y;
  pose.pose.position.z    = 0.0;
  pose.pose.orientation.w = 1.0;

  setOutput("output_pose", pose);

  RCLCPP_INFO(rclcpp::get_logger("LockTarget"),
    "목표 좌표 고정: x=%.3f, y=%.3f", point.x, point.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sar_bt_mission
