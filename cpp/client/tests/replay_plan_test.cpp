// 阶段四 D：Replay 默认只转发 Simulator 输出，绝不把轮控命令送回实时 topic。
#include "slope_sim/client/replay_plan.hpp"

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

}  // namespace

int main() {
  using slope_sim::client::v2::PlanReplayFrame;
  using slope_sim::client::v2::RecordedRawFrame;

  const RecordedRawFrame state{
      "/sim/wheel/state", {std::byte{0x21}, std::byte{0x22}}, 3, 1000, 900};
  const auto replay = PlanReplayFrame(state);
  Require(replay.topic == "/replay/sim/wheel/state", "Replay did not isolate the output topic");
  Require(replay.payload == state.payload && replay.sequence == state.sequence &&
              replay.log_time_ns == state.log_time_ns && replay.publish_time_ns == state.publish_time_ns,
          "Replay changed the validated raw frame");

  bool rejected_command = false;
  try {
    static_cast<void>(PlanReplayFrame({"/sim/wheel/command", {std::byte{0x01}}, 1, 100, 100}));
  } catch (const std::exception&) {
    rejected_command = true;
  }
  Require(rejected_command, "Replay accepted a WheelCommand frame by default");
  return 0;
}
