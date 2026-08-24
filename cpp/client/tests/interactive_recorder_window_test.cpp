// 阶段四 Task 5：锁定人工录制在完整 10 Hz 边界开始、停止与故障时的提交范围。
#include "slope_sim/client/interactive_recorder_window.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {

using slope_sim::client::v2::InteractiveRecorderWindow;
using slope_sim::client::v2::RecordedRawFrame;

void Require(bool condition, std::string_view message) {
  if (!condition) throw std::runtime_error(std::string(message));
}

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

std::vector<RecordedRawFrame> CompleteSensorBoundary(InteractiveRecorderWindow& window,
                                                     std::uint64_t timestamp_ns) {
  std::vector<RecordedRawFrame> committed;
  for (std::size_t topic = 2; topic < 5; ++topic) {
    const auto next = window.Observe(topic, Frame(topic, timestamp_ns));
    committed.insert(committed.end(), next.begin(), next.end());
  }
  return committed;
}

}  // namespace

int main() {
  InteractiveRecorderWindow window;
  Require(window.state() == InteractiveRecorderWindow::State::kAwaitingStart,
          "window did not begin awaiting start");
  Require(window.RequestStart(), "window rejected a valid start request");

  // 起始传感器边界之前的高频帧不得泄漏；三条同刻传感器帧才授权录制。
  Require(window.Observe(0, Frame(0, 90)).empty(), "partial boundary leaked into MCAP");
  Require(window.Observe(1, Frame(1, 90)).empty(), "partial boundary leaked into MCAP");
  const auto started = CompleteBoundary(window, 100);
  Require(started.size() == 3, "complete start boundary did not commit three sensor frames");
  for (const auto& frame : started) {
    Require(frame.log_time_ns == 100, "start committed a preceding partial frame");
  }
  Require(window.state() == InteractiveRecorderWindow::State::kRecording,
          "complete boundary did not enter recording");

  // 两条高频 topic 保持各自时钟域，在已授权窗口内即时写入。
  Require(window.Observe(0, Frame(0, 110)).size() == 1, "command was not recorded after start");
  Require(window.Observe(1, Frame(1, 110)).size() == 1, "wheel state was not recorded after start");
  Require(window.RequestStop(), "window rejected a valid stop request");
  Require(window.state() == InteractiveRecorderWindow::State::kAwaitingFinalBoundary,
          "window did not await final boundary");
  const auto stopped = CompleteBoundary(window, 200);
  Require(stopped.size() == 5, "final boundary did not commit all final-window frames");
  for (const auto& frame : stopped) {
    Require(frame.log_time_ns <= 200, "final boundary committed a future frame");
  }
  Require(window.state() == InteractiveRecorderWindow::State::kReadyToFinalize,
          "final boundary did not finalize the window");
  Require(!window.RequestStart(), "finalized window accepted another start");
  Require(!window.RequestStop(), "finalized window accepted another stop");

  InteractiveRecorderWindow late_frame;
  Require(late_frame.RequestStart(), "late-frame window rejected start");
  Require(CompleteBoundary(late_frame, 100).size() == 3,
          "late-frame window did not commit initial boundary");
  // eCAL callback may deliver a 100 Hz frame only after the enclosing 10 Hz
  // boundary was committed. Keep that violation diagnosable for the recorder.
  Require(late_frame.Observe(1, Frame(1, 90)).empty(), "late frame committed unexpectedly");
  Require(late_frame.state() == InteractiveRecorderWindow::State::kSafeStopRequired,
          "late frame did not require safe stop");
  Require(late_frame.failure_reason() == "wheel state arrived after its sensor boundary",
          "late frame did not preserve its failure classification");

  InteractiveRecorderWindow delayed_sensor;
  Require(delayed_sensor.RequestStart(), "delayed-sensor window rejected start");
  Require(CompleteBoundary(delayed_sensor, 100).size() == 3,
          "delayed-sensor window did not commit its start boundary");
  // Command and WheelState publish at 100 Hz while LiDAR can arrive from an
  // asynchronous worker. The bounded recorder window is measured in 10 Hz
  // sensor periods, not raw 100 Hz timestamps.
  for (std::uint64_t timestamp_ns = 110; timestamp_ns <= 430; timestamp_ns += 10) {
    Require(delayed_sensor.Observe(0, Frame(0, timestamp_ns)).size() == 1,
            "high-rate command was not recorded during the sensor interval");
    Require(delayed_sensor.Observe(1, Frame(1, timestamp_ns)).size() == 1,
            "high-rate wheel state was not recorded during the sensor interval");
  }
  std::vector<RecordedRawFrame> delayed_committed;
  for (std::size_t topic = 2; topic < 5; ++topic) {
    const auto next = delayed_sensor.Observe(topic, Frame(topic, 200));
    delayed_committed.insert(delayed_committed.end(), next.begin(), next.end());
  }
  Require(delayed_sensor.state() == InteractiveRecorderWindow::State::kRecording,
          "asynchronous sensor latency exhausted the 10 Hz recorder window");
  Require(delayed_committed.size() == 3,
          "delayed sensor boundary did not commit its three sensor frames");

  InteractiveRecorderWindow faulted;
  Require(faulted.RequestStart(), "faulted window rejected start");
  faulted.RequireSafeStop();
  Require(faulted.state() == InteractiveRecorderWindow::State::kSafeStopRequired,
          "explicit safe stop was not latched");
  Require(faulted.Observe(0, Frame(0, 100)).empty(), "safe-stopped window accepted a frame");
  Require(!faulted.RequestStop(), "safe-stopped window accepted stop");

  // Command uses a wall-clock timestamp and WheelState uses the simulation clock;
  // only the three sensor outputs share a 10 Hz sample time.  The interactive
  // boundary must not attempt to construct an impossible five-topic timestamp.
  InteractiveRecorderWindow mixed_clocks;
  Require(mixed_clocks.RequestStart(), "mixed-clock window rejected start");
  Require(mixed_clocks.Observe(0, Frame(0, 1700000000000000000ULL)).empty(),
          "pre-start wall-clock command leaked into MCAP");
  Require(mixed_clocks.Observe(1, Frame(1, 10)).empty(),
          "pre-start wheel state leaked into MCAP");
  const auto mixed_started = CompleteSensorBoundary(mixed_clocks, 100000000);
  Require(mixed_started.size() == 3,
          "same-time sensor boundary did not start mixed-clock capture");
  Require(mixed_clocks.state() == InteractiveRecorderWindow::State::kRecording,
          "sensor boundary did not enter mixed-clock recording");
  const auto command = mixed_clocks.Observe(0, Frame(0, 1700000000010000000ULL));
  Require(command.size() == 1 && command.front().topic == "/topic/0",
          "recording window did not preserve a wall-clock command");
  const auto wheel = mixed_clocks.Observe(1, Frame(1, 110000000));
  Require(wheel.size() == 1 && wheel.front().topic == "/topic/1",
          "recording window did not preserve a simulation-clock wheel state");
  Require(mixed_clocks.RequestStop(), "mixed-clock window rejected stop");
  const auto mixed_stopped = CompleteSensorBoundary(mixed_clocks, 200000000);
  Require(mixed_stopped.size() == 3,
          "same-time sensor boundary did not finalize mixed-clock capture");
  Require(mixed_clocks.state() == InteractiveRecorderWindow::State::kReadyToFinalize,
          "mixed-clock final sensor boundary did not finalize capture");
  return 0;
}
