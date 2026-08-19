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
  if (!AddLocked(topic_index, std::move(frame))) return {};
  auto boundary = FirstCompleteBoundaryLocked();
  if (boundary == pending_.end()) return {};
  const std::uint64_t timestamp_ns = boundary->first;
  if (last_committed_timestamp_ns_ == 0) {
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
  RequireSafeStopLocked();
}

InteractiveRecorderWindow::State InteractiveRecorderWindow::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool InteractiveRecorderWindow::AddLocked(std::size_t topic_index, RecordedRawFrame frame) {
  if (topic_index >= 5 || frame.log_time_ns == 0 || frame.log_time_ns != frame.publish_time_ns ||
      frame.log_time_ns <= last_committed_timestamp_ns_) {
    RequireSafeStopLocked();
    return false;
  }
  auto [boundary, inserted] = pending_.try_emplace(frame.log_time_ns);
  if (!inserted && boundary->second.received[topic_index]) {
    RequireSafeStopLocked();
    return false;
  }
  boundary->second.received[topic_index] = true;
  boundary->second.frames.push_back(std::move(frame));
  if (pending_.size() > kMaximumPendingBoundaries) {
    RequireSafeStopLocked();
    return false;
  }
  return true;
}

std::map<std::uint64_t, InteractiveRecorderWindow::PendingBoundary>::iterator
InteractiveRecorderWindow::FirstCompleteBoundaryLocked() {
  for (auto boundary = pending_.begin(); boundary != pending_.end(); ++boundary) {
    bool complete = true;
    for (const bool received : boundary->second.received) complete = complete && received;
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
  last_committed_timestamp_ns_ = timestamp_ns;
  return committed;
}

void InteractiveRecorderWindow::RequireSafeStopLocked() {
  pending_.clear();
  state_ = State::kSafeStopRequired;
}

}  // namespace slope_sim::client::v2
