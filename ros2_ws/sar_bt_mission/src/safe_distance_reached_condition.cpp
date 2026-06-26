#include "sar_bt_mission/safe_distance_reached_condition.hpp"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace sar_bt_mission {

SafeDistanceReachedCondition::SafeDistanceReachedCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config), node_(node)
{
  // SensorDataQoS: 최신 스캔 한 개만 유지하는 Best-Effort QoS
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      latest_scan_ = msg;
    });

  RCLCPP_INFO(node_->get_logger(), "SafeDistanceReached: /scan 구독 시작");
}

BT::PortsList SafeDistanceReachedCondition::providedPorts()
{
  return {
    BT::InputPort<double>("min_distance", 1.0,
      "요구조자 접근 최소 거리 [m] — Nav2 목표 도달 후 이 거리 이상이면 SUCCESS"),
  };
}

BT::NodeStatus SafeDistanceReachedCondition::tick()
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  // 스캔 데이터 미수신 시 경고 후 SUCCESS (데모 환경에서 LiDAR 없을 경우 대비)
  if (!scan) {
    RCLCPP_WARN_ONCE(node_->get_logger(),
      "SafeDistanceReached: /scan 미수신 — 데이터 없이 SUCCESS 처리 (LiDAR 연결 확인)");
    return BT::NodeStatus::SUCCESS;
  }

  double min_dist_threshold = 1.0;
  getInput("min_distance", min_dist_threshold);

  // 전방 ±15° 범위에서 유효한 최소 거리 계산
  static constexpr float FRONT_HALF_ANGLE = 15.0f * static_cast<float>(M_PI) / 180.0f;
  float front_min = std::numeric_limits<float>::max();

  const std::size_t n = scan->ranges.size();
  for (std::size_t i = 0; i < n; ++i) {
    const float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
    if (std::abs(angle) <= FRONT_HALF_ANGLE) {
      const float r = scan->ranges[i];
      if (std::isfinite(r) && r > scan->range_min && r < scan->range_max) {
        front_min = std::min(front_min, r);
      }
    }
  }

  if (front_min == std::numeric_limits<float>::max()) {
    // 전방 유효 데이터 없음 → 안전 처리
    RCLCPP_WARN(node_->get_logger(), "SafeDistanceReached: 전방 유효 스캔 없음 → SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  const bool safe = (static_cast<double>(front_min) >= min_dist_threshold);
  RCLCPP_DEBUG(node_->get_logger(),
    "SafeDistanceReached: 전방 최소거리=%.3fm, 임계값=%.3fm → %s",
    front_min, min_dist_threshold, safe ? "SUCCESS" : "FAILURE");

  return safe ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace sar_bt_mission
