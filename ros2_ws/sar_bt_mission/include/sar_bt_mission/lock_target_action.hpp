#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sar_bt_mission {

/**
 * @brief current_target_point(Point)를 locked_target_pose(PoseStamped)로 변환·고정
 *
 * 검출 좌표가 계속 갱신될 수 있으므로 구조 진입 직전에 스냅샷을 찍어 고정한다.
 * frame_id="map", z=0, orientation.w=1 으로 변환하여 Nav2에 전달 가능한 형태로 만든다.
 */
class LockTargetAction : public BT::SyncActionNode
{
public:
  LockTargetAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace sar_bt_mission
