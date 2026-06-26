#include "sar_bt_mission/publish_rescue_report_action.hpp"

#include <sstream>
#include <iomanip>

namespace sar_bt_mission {

PublishRescueReportAction::PublishRescueReportAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  report_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/rescue_report", rclcpp::SystemDefaultsQoS());
}

BT::PortsList PublishRescueReportAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose",
      "구조 위치 (locked_target_pose)"),
  };
}

BT::NodeStatus PublishRescueReportAction::tick()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!getInput("target_pose", pose)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[PublishRescueReport] target_pose 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }

  const double ts_sec = static_cast<double>(node_->now().nanoseconds()) * 1e-9;

  std::ostringstream json;
  json << std::fixed << std::setprecision(6);
  json << "{\n";
  json << "  \"event\": \"RESCUE_REPORT\",\n";
  json << "  \"timestamp_sec\": " << ts_sec << ",\n";
  json << "  \"position\": {\n";
  json << "    \"x\": " << pose.pose.position.x << ",\n";
  json << "    \"y\": " << pose.pose.position.y << ",\n";
  json << "    \"z\": " << pose.pose.position.z << "\n";
  json << "  },\n";
  json << "  \"status\": \"RESCUED\"\n";
  json << "}";

  std_msgs::msg::String report_msg;
  report_msg.data = json.str();
  report_pub_->publish(report_msg);

  RCLCPP_INFO(node_->get_logger(),
    "[PublishRescueReport] 보고서 발행 완료 — x:%.3f, y:%.3f",
    pose.pose.position.x, pose.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sar_bt_mission
