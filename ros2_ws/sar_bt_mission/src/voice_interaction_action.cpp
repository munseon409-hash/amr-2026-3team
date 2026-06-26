#include "sar_bt_mission/voice_interaction_action.hpp"

namespace sar_bt_mission {

VoiceInteractionAction::VoiceInteractionAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
  voice_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/voice_output", rclcpp::SystemDefaultsQoS());
}

BT::PortsList VoiceInteractionAction::providedPorts()
{
  return {};
}

BT::NodeStatus VoiceInteractionAction::tick()
{
  std_msgs::msg::String msg;
  msg.data = "구조대가 오고 있습니다. 안심하세요.";
  voice_pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "[VoiceInteraction] %s", msg.data.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sar_bt_mission
