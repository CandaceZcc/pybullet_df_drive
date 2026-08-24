// 阶段四 Task 5：把人工采集的完整 10 Hz 起止边界与写盘队列隔离。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "slope_sim/client/recorder_queue.hpp"

namespace slope_sim::client::v2 {

/// 三条传感器以同刻 10 Hz 边界开关窗口；Command/WheelState 保持各自时钟域。
class InteractiveRecorderWindow final {
 public:
  enum class State {
    kAwaitingStart,
    kAwaitingStartBoundary,
    kRecording,
    kAwaitingFinalBoundary,
    kReadyToFinalize,
    kSafeStopRequired,
  };

  /// 开始请求在下一完整三传感器、10 Hz 时刻生效。
  bool RequestStart();
  /// 停止请求在下一完整三传感器、10 Hz 时刻后生效。
  bool RequestStop();
  /// 返回现在可以交给 RecorderSession 的连续原始帧；未对齐时只在有界内存中暂存。
  std::vector<RecordedRawFrame> Observe(std::size_t topic_index, RecordedRawFrame frame);
  /// 不可恢复的控制/传输故障取消暂存内容，防止以后发布为完整会话。
  void RequireSafeStop();
  [[nodiscard]] State state() const noexcept;
  /// 返回最近一次 fail-closed 的分类，供编排层保留有界诊断。
  [[nodiscard]] std::string failure_reason() const;

 private:
  struct PendingBoundary final {
    std::array<bool, 5> received{};
    std::vector<RecordedRawFrame> frames;
  };

  static constexpr std::size_t kMaximumPendingSensorBoundaries = 32;

  bool AddLocked(std::size_t topic_index, RecordedRawFrame frame);
  std::map<std::uint64_t, PendingBoundary>::iterator FirstCompleteBoundaryLocked();
  std::vector<RecordedRawFrame> CommitThroughLocked(std::uint64_t timestamp_ns);
  void RequireSafeStopLocked(std::string_view reason);

  mutable std::mutex mutex_;
  State state_ = State::kAwaitingStart;
  std::uint64_t last_sensor_boundary_timestamp_ns_ = 0;
  std::map<std::uint64_t, PendingBoundary> pending_;
  std::string failure_reason_;
};

}  // namespace slope_sim::client::v2
