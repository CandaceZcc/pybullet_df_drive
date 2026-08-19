// 阶段四 Task 5：锁定人工录制在完整 10 Hz 边界开始、停止与故障时的提交范围。
#include "slope_sim/client/interactive_recorder_window.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using slope_sim::client::v2::InteractiveRecorderWindow;
using slope_sim::client::v2::RecordedRawFrame;

RecordedRawFrame Frame(std::size_t topic, std::uint64_t timestamp_ns) {
  return {"/topic/" + std::to_string(topic), {std::byte{static_cast<unsigned char>(topic)}},
          static_cast<std::uint32_t>(timestamp_ns / 10), timestamp_ns, timestamp_ns};
}

std::vector<RecordedRawFrame> CompleteBoundary(InteractiveRecorderWindow& window,
                                               std::uint64_t timestamp_ns) {
  std::vector<RecordedRawFrame> committed;
  for (std::size_t topic = 0; topic < 5; ++topic) {
    const auto next = window.Observe(topic, Frame(topic, timestamp_ns));
    committed.insert(committed.end(), next.begin(), next.end());
  }
  return committed;
}

}  // namespace

int main() {
  InteractiveRecorderWindow window;
  assert(window.state() == InteractiveRecorderWindow::State::kAwaitingStart);
  assert(window.RequestStart());

  // 非完整 10 Hz 片段不得泄漏到 MCAP；完整五 topic 同刻到齐才开始提交。
  assert(window.Observe(0, Frame(0, 90)).empty());
  assert(window.Observe(1, Frame(1, 90)).empty());
  const auto started = CompleteBoundary(window, 100);
  assert(started.size() == 5);
  for (const auto& frame : started) assert(frame.log_time_ns == 100);
  assert(window.state() == InteractiveRecorderWindow::State::kRecording);

  // 100 Hz 的 Command/Wheel 在下一完整边界前保持暂存，不能抢先 finalize。
  assert(window.Observe(0, Frame(0, 110)).empty());
  assert(window.Observe(1, Frame(1, 110)).empty());
  assert(window.RequestStop());
  assert(window.state() == InteractiveRecorderWindow::State::kAwaitingFinalBoundary);
  const auto stopped = CompleteBoundary(window, 200);
  assert(stopped.size() == 7);
  for (const auto& frame : stopped) assert(frame.log_time_ns <= 200);
  assert(window.state() == InteractiveRecorderWindow::State::kReadyToFinalize);
  assert(!window.RequestStart());
  assert(!window.RequestStop());

  InteractiveRecorderWindow faulted;
  assert(faulted.RequestStart());
  faulted.RequireSafeStop();
  assert(faulted.state() == InteractiveRecorderWindow::State::kSafeStopRequired);
  assert(faulted.Observe(0, Frame(0, 100)).empty());
  assert(!faulted.RequestStop());
  return 0;
}
