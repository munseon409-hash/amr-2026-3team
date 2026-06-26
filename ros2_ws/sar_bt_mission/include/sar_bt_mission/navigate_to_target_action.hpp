#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

/**
 * @brief locked_target_pose를 목표로 Nav2 주행을 실행하는 RosActionNode
 *
 * BT::RosActionNode 기반으로 구현하여:
 *   - Halt 시 Nav2 CancelGoal 자동 호출
 *   - Nav2 내장 복구(Backup, Spin, ClearCostmap)를 활용
 *   - 주행 결과(SUCCEEDED/ABORTED/CANCELED)를 BT 상태로 매핑
 */
class NavigateToTargetAction
  : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToTargetAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // providedBasicPorts()로 action_name 포트를 자동 포함
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "target_pose", "Navigation goal pose (locked_target_pose)")
    });
  }

  bool setGoal(Goal & goal) override
  {
    geometry_msgs::msg::PoseStamped pose;
    if (!getInput("target_pose", pose)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToTarget"),
        "target_pose 포트 읽기 실패 — Blackboard에 locked_target_pose가 설정되어 있는지 확인");
      return false;
    }
    goal.pose = pose;
    // behavior_tree 필드를 비워두면 Nav2가 기본 BT를 사용 (내장 복구 포함)
    goal.behavior_tree = "";
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToTarget"), "목표 지점 도달 성공");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(rclcpp::get_logger("NavigateToTarget"),
      "주행 실패 (ResultCode=%d)", static_cast<int>(wr.code));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToTarget"),
      "NavigateToTarget 오류: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace sar_bt_mission
