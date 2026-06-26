#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

/**
 * @brief 순찰 웨이포인트(WP1, WP2)로 이동하는 RosActionNode
 *
 * NavigateToTargetAction과 동일한 Nav2 연동 구조이며,
 * XML에서 name="MoveToWP1" / name="MoveToWP2"로 구별한다.
 * ReactiveFallback이 Emergency 브랜치로 전환할 때 Halt → CancelGoal 자동 호출.
 */
class NavigateToWaypointAction
  : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToWaypointAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "waypoint_pose", "Patrol waypoint pose (waypoint_1 or waypoint_2)")
    });
  }

  bool setGoal(Goal & goal) override
  {
    geometry_msgs::msg::PoseStamped pose;
    if (!getInput("waypoint_pose", pose)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
        "waypoint_pose 포트 읽기 실패 — main.cpp에서 waypoint_1/2가 설정되어 있는지 확인");
      return false;
    }
    goal.pose = pose;
    goal.behavior_tree = "";
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
        "[%s] 웨이포인트 도달", name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(rclcpp::get_logger("NavigateToWaypoint"),
      "[%s] 웨이포인트 이동 실패 (ResultCode=%d)",
      name().c_str(), static_cast<int>(wr.code));
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
      "[%s] 오류: %s", name().c_str(), BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace sar_bt_mission
