// 阶段四 C1：以编译态测试锁定 C++ SDK 的五 topic 元数据。
#include "slope_sim/client/v2_topics.hpp"

#include <array>
#include <cassert>
#include <string_view>

int main() {
  using slope_sim::client::v2::Direction;
  using slope_sim::client::v2::TopicContract;
  using slope_sim::client::v2::TopicContracts;

  constexpr std::array<TopicContract, 5> expected{{
      {"/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, Direction::kSubscribe},
      {"/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, Direction::kPublish},
      {"/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, Direction::kPublish},
      {"/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, Direction::kPublish},
      {"/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, Direction::kPublish},
  }};

  const auto& actual = TopicContracts();
  assert(actual.size() == expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    assert(std::string_view(actual[index].topic) == expected[index].topic);
    assert(std::string_view(actual[index].type_name) == expected[index].type_name);
    assert(actual[index].rate_hz == expected[index].rate_hz);
    assert(actual[index].direction == expected[index].direction);
  }
  return 0;
}
