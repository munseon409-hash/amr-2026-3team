#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sar_bt_mission {

class VoiceInteractionAction : public BT::SyncActionNode
{
public:
  VoiceInteractionAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;
};

}  // namespace sar_bt_mission
