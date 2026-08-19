// 阶段四 C1：冻结五 topic 元数据的唯一 C++ SDK 定义。
#include "slope_sim/client/v2_topics.hpp"

namespace slope_sim::client::v2 {

const std::array<TopicContract, 5>& TopicContracts() {
  // 必须与 slope_sim.interfaces.v2.topics.V2_TOPICS 保持逐项一致。
  static constexpr std::array<TopicContract, 5> kContracts{{
      {"/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, Direction::kSubscribe},
      {"/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, Direction::kPublish},
      {"/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, Direction::kPublish},
      {"/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, Direction::kPublish},
      {"/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, Direction::kPublish},
  }};
  return kContracts;
}

}  // namespace slope_sim::client::v2
