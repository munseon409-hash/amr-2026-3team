#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sar_bt_mission {

/**
 * @brief 구조 완료 보고서를 JSON 형태의 ROS 토픽으로 발행하는 ActionNode
 *
 * 발행 토픽: /rescue_report (std_msgs/String, JSON 포맷)
 *
 * JSON 예시:
 * {
 *   "event": "RESCUE_REPORT",
 *   "timestamp_sec": 1234567890.123,
 *   "position": { "x": 1.5, "y": 2.3, "z": 0.0 },
 *   "status": "RESCUED"
 * }
 */
class PublishRescueReportAction : public BT::SyncActionNode
{
public:
  PublishRescueReportAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
};

}  // namespace sar_bt_mission
