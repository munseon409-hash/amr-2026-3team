#include "sar_bt_mission/align_to_target_action.hpp"

#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sar_bt_mission {

AlignToTargetAction::AlignToTargetAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList AlignToTargetAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>("target_point",
      "화면 정규화 좌표 (x: -1.0 ~ 1.0)"),
    BT::InputPort<double>("project_distance", 1.5,
      "정렬 완료 후 전방 투영 거리 (m)"),
    BT::InputPort<double>("align_threshold", 0.1,
      "정렬 완료 판단 임계값 (|x| < threshold)"),
    BT::InputPort<double>("kp", 0.5,
      "P 제어 각속도 게인"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose",
      "Nav2에 전달할 맵 좌표 목표 (locked_target_pose)"),
  };
}

BT::NodeStatus AlignToTargetAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[AlignToTarget] 정렬 시작");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignToTargetAction::onRunning()
{
  geometry_msgs::msg::Point target_point;
  if (!getInput("target_point", target_point)) {
    RCLCPP_ERROR(node_->get_logger(), "[AlignToTarget] target_point 읽기 실패");
    return BT::NodeStatus::FAILURE;
  }

  double align_threshold = 0.1;
  double kp = 0.5;
  double project_distance = 1.5;
  getInput("align_threshold", align_threshold);
  getInput("kp", kp);
  getInput("project_distance", project_distance);

  const double x_err = target_point.x;

  if (std::abs(x_err) > align_threshold) {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = -kp * x_err;
    cmd_vel_pub_->publish(cmd);

    RCLCPP_DEBUG(node_->get_logger(),
      "[AlignToTarget] 정렬 중 x_err=%.3f, angular.z=%.3f", x_err, cmd.angular.z);
    return BT::NodeStatus::RUNNING;
  }

  // 정렬 완료 → 로봇 정지
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

  // TF에서 현재 맵 좌표 + yaw 읽기
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      "map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[AlignToTarget] TF 읽기 실패: %s — 재시도", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double yaw = tf2::getYaw(tf_stamped.transform.rotation);
  const double cx  = tf_stamped.transform.translation.x;
  const double cy  = tf_stamped.transform.translation.y;

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id    = "map";
  goal.header.stamp       = node_->now();
  goal.pose.position.x    = cx + project_distance * std::cos(yaw);
  goal.pose.position.y    = cy + project_distance * std::sin(yaw);
  goal.pose.position.z    = 0.0;
  goal.pose.orientation.w = std::cos(yaw / 2.0);
  goal.pose.orientation.z = std::sin(yaw / 2.0);

  setOutput("output_pose", goal);

  RCLCPP_INFO(node_->get_logger(),
    "[AlignToTarget] 정렬 완료 — 목표: (%.3f, %.3f) yaw=%.3f",
    goal.pose.position.x, goal.pose.position.y, yaw);

  return BT::NodeStatus::SUCCESS;
}

void AlignToTargetAction::onHalted()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  RCLCPP_WARN(node_->get_logger(), "[AlignToTarget] 중단됨 — 로봇 정지");
}

}  // namespace sar_bt_mission
