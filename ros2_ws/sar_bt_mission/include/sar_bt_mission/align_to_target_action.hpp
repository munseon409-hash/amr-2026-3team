#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace sar_bt_mission {

/**
 * @brief 화면 정규화 좌표(x: -1~1)로 로봇을 정렬한 뒤 전방 지점을 맵 좌표로 투영
 *
 * 동작 순서:
 *   1. /red_target_point.x 오차를 줄이도록 제자리 회전 (P 제어)
 *   2. |x| < align_threshold 달성 시 TF에서 현재 yaw 읽기
 *   3. 현재 위치 + 헤딩 방향으로 project_distance(m) 전방 지점을 PoseStamped로 출력
 *   4. SUCCESS 반환 → NavigateToTarget이 이 좌표로 Nav2 주행
 */
class AlignToTargetAction : public BT::StatefulActionNode
{
public:
  AlignToTargetAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sar_bt_mission
