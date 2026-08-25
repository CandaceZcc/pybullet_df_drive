// 阶段四 Task 5：实现人工 Recorder 的有界、同刻五 topic 起止窗口。
#include "slope_sim/client/interactive_recorder_window.hpp"

#include <iterator>
#include <utility>

namespace slope_sim::client::v2 {

bool InteractiveRecorderWindow::RequestStart() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::kAwaitingStart) return false;
  // RequestStart 本身只改变授权状态；首个完整边界仍由 Observe 原子选择。
  state_ = State::kAwaitingStartBoundary;
  return true;
}

bool InteractiveRecorderWindow::RequestStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::kRecording) return false;
  state_ = State::kAwaitingFinalBoundary;
  return true;
}

std::vector<RecordedRawFrame> InteractiveRecorderWindow::Observe(
    std::size_t topic_index, RecordedRawFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == State::kAwaitingStart || state_ == State::kReadyToFinalize ||
      state_ == State::kSafeStopRequired) {
    return {};
  }
  if (topic_index < 2) {
    if (frame.log_time_ns == 0 || frame.log_time_ns != frame.publish_time_ns) {
      RequireSafeStopLocked("high-rate frame timestamp is invalid");
      return {};
    }
    // Command uses a wall clock while WheelState uses simulation time.  Neither
    // can participate in the sensor timestamp boundary, so they are accepted
    // only after that boundary authorizes recording.
    if (state_ == State::kAwaitingStartBoundary) return {};
    if (topic_index == 0) return {std::move(frame)};
    if (frame.log_time_ns <= last_sensor_boundary_timestamp_ns_) {
      ++dropped_late_wheel_state_count_;
      return {};
    }
    const auto [entry, inserted] = pending_wheel_states_.try_emplace(
        frame.log_time_ns, std::move(frame));
    if (!inserted) {
      RequireSafeStopLocked("duplicate WheelState timestamp in bounded reorder window");
      return {};
    }
    if (pending_wheel_states_.size() > kMaximumPendingWheelStates) {
      RequireSafeStopLocked("pending WheelState frames exceeded the bounded reorder window");
    }
    return {};
  }
  if (!AddLocked(topic_index, std::move(frame))) return {};
  auto boundary = FirstCompleteBoundaryLocked();
  if (boundary == pending_.end()) return {};
  const std::uint64_t timestamp_ns = boundary->first;
  if (last_sensor_boundary_timestamp_ns_ == 0) {
    // 起始边界之前的帧不属于用户已经授权的采集区间。
    pending_.erase(pending_.begin(), boundary);
  }
  const auto committed = CommitThroughLocked(timestamp_ns);
  if (state_ == State::kAwaitingStartBoundary) state_ = State::kRecording;
  if (state_ == State::kAwaitingFinalBoundary) state_ = State::kReadyToFinalize;
  return committed;
}

void InteractiveRecorderWindow::RequireSafeStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  RequireSafeStopLocked("safe stop requested by recorder");
}

InteractiveRecorderWindow::State InteractiveRecorderWindow::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string InteractiveRecorderWindow::failure_reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return failure_reason_;
}

std::size_t InteractiveRecorderWindow::dropped_late_wheel_state_count() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_late_wheel_state_count_;
}

bool InteractiveRecorderWindow::AddLocked(std::size_t topic_index, RecordedRawFrame frame) {
  if (topic_index >= 5) {
    RequireSafeStopLocked("topic index is outside the frozen recorder contract");
    return false;
  }
  if (frame.log_time_ns == 0) {
    RequireSafeStopLocked("frame timestamp is zero");
    return false;
  }
  if (frame.log_time_ns != frame.publish_time_ns) {
    RequireSafeStopLocked("frame log and publish timestamps differ");
    return false;
  }
  if (frame.log_time_ns <= last_sensor_boundary_timestamp_ns_) {
    RequireSafeStopLocked("frame arrived after its committed boundary");
    return false;
  }
  auto [boundary, inserted] = pending_.try_emplace(frame.log_time_ns);
  if (!inserted && boundary->second.received[topic_index]) {
    RequireSafeStopLocked("duplicate topic frame in one timestamp boundary");
    return false;
  }
  boundary->second.received[topic_index] = true;
  boundary->second.frames.push_back(std::move(frame));
  if (pending_.size() > kMaximumPendingSensorBoundaries) {
    RequireSafeStopLocked("pending timestamp boundaries exceeded the bounded 10Hz window");
    return false;
  }
  return true;
}

std::map<std::uint64_t, InteractiveRecorderWindow::PendingBoundary>::iterator
InteractiveRecorderWindow::FirstCompleteBoundaryLocked() {
  for (auto boundary = pending_.begin(); boundary != pending_.end(); ++boundary) {
    bool complete = true;
    for (std::size_t topic_index = 2; topic_index < 5; ++topic_index) {
      complete = complete && boundary->second.received[topic_index];
    }
    if (complete) return boundary;
  }
  return pending_.end();
}

std::vector<RecordedRawFrame> InteractiveRecorderWindow::CommitThroughLocked(std::uint64_t timestamp_ns) {
  std::vector<RecordedRawFrame> committed;
  auto end = pending_.upper_bound(timestamp_ns);
  for (auto entry = pending_.begin(); entry != end; ++entry) {
    auto& frames = entry->second.frames;
    committed.insert(committed.end(), std::make_move_iterator(frames.begin()),
                     std::make_move_iterator(frames.end()));
  }
  pending_.erase(pending_.begin(), end);
  const auto wheel_end = pending_wheel_states_.upper_bound(timestamp_ns);
  for (auto entry = pending_wheel_states_.begin(); entry != wheel_end; ++entry) {
    committed.push_back(std::move(entry->second));
  }
  pending_wheel_states_.erase(pending_wheel_states_.begin(), wheel_end);
  last_sensor_boundary_timestamp_ns_ = timestamp_ns;
  return committed;
}

void InteractiveRecorderWindow::RequireSafeStopLocked(std::string_view reason) {
  pending_.clear();
  pending_wheel_states_.clear();
  failure_reason_.assign(reason);
  state_ = State::kSafeStopRequired;
}

}  // namespace slope_sim::client::v2
