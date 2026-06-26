#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <vector>

namespace sar_bt_mission {

/**
 * @brief 구조 완료된 target_id를 served_target_ids 벡터에 추가하는 ActionNode
 *
 * Blackboard 읽기/쓰기 패턴:
 *   in_served_ids  (InputPort)  → 현재 served_target_ids 값 읽기
 *   out_served_ids (OutputPort) → target_id 추가 후 served_target_ids에 다시 쓰기
 *
 * XML에서 두 포트를 동일한 BB 키({served_target_ids})로 연결하여
 * 읽기-수정-쓰기(read-modify-write)를 원자적으로 수행한다.
 */
class MarkTargetServedAction : public BT::SyncActionNode
{
public:
  MarkTargetServedAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace sar_bt_mission
