#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <vector>

namespace sar_bt_mission {

/**
 * @brief 검출된 요구조자가 실제 구조 대상인지 검증하는 조건 노드
 *
 * 아래 세 조건을 모두 만족해야 SUCCESS:
 *   1. confidence > CONFIDENCE_THRESHOLD (0.7)
 *   2. bbox_area   > BBOX_AREA_THRESHOLD  (5000 px²)
 *   3. target_id가 served_target_ids에 없을 것 (중복 구조 방지)
 */
class TargetConfirmedCondition : public BT::ConditionNode
{
public:
  static constexpr float CONFIDENCE_THRESHOLD = 0.7f;
  static constexpr float BBOX_AREA_THRESHOLD  = 5000.0f;

  TargetConfirmedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace sar_bt_mission
