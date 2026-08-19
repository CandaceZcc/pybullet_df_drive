// 阶段四 C2：锁定 Recorder 队列的 overflow 不丢弃既有原始帧。
#include "slope_sim/client/recorder_queue.hpp"

#include <cassert>

int main() {
  using slope_sim::client::v2::RecordedRawFrame;
  using slope_sim::client::v2::RecorderEnqueueResult;
  using slope_sim::client::v2::RecorderQueue;

  RecorderQueue queue(1);
  const RecordedRawFrame first{"/sim/wheel/state", {std::byte{0x01}, std::byte{0x02}}};
  const RecordedRawFrame second{"/sim/imu/attitude", {std::byte{0x03}}};
  assert(queue.Enqueue(first) == RecorderEnqueueResult::kAccepted);
  assert(queue.Enqueue(second) == RecorderEnqueueResult::kOverflow);
  assert(queue.size() == 1);
  assert(queue.Pop().payload == first.payload);
  assert(queue.Enqueue(second) == RecorderEnqueueResult::kAccepted);
  assert(queue.Pop().topic == second.topic);
  return 0;
}
