// 阶段四 D：Replay 的纯 topic 隔离判据，避免任何原始 command 回流到 /sim/*。
#include "slope_sim/client/replay_plan.hpp"

#include <stdexcept>
#include <string>

#include "slope_sim/client/v2_topics.hpp"

namespace slope_sim::client::v2 {

RecordedRawFrame PlanReplayFrame(const RecordedRawFrame& frame) {
  for (const auto& contract : TopicContracts()) {
    if (frame.topic != contract.topic) continue;
    if (contract.direction != Direction::kPublish) {
      throw std::invalid_argument("WheelCommand replay requires an explicit isolated shadow world");
    }
    return {
        std::string("/replay") + frame.topic,
        frame.payload,
        frame.sequence,
        frame.log_time_ns,
        frame.publish_time_ns,
    };
  }
  throw std::invalid_argument("Replay frame is not in the frozen v2 contract");
}

}  // namespace slope_sim::client::v2
