#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <limits>
#include <mutex>

namespace sar_bt_mission {

/**
 * @brief 전방 LiDAR 거리로 요구조자에게 안전 접근 거리를 확인하는 조건 노드
 *
 * /scan 토픽을 구독하고, 전방 ±15° 범위 내 최소 거리를 min_distance 임계값과 비교.
 * - SUCCESS: 최소 거리 >= min_distance (목표 직전 정지 확인)
 * - FAILURE: 최소 거리 <  min_distance (아직 접근 중)
 *
 * 스캔 데이터가 없으면 데모 안전을 위해 SUCCESS를 반환하고 경고를 출력한다.
 */
class SafeDistanceReachedCondition : public BT::ConditionNode
{
public:
  SafeDistanceReachedCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

}  // namespace sar_bt_mission
