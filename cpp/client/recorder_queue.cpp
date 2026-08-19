// 阶段四 C2：实现 Recorder 的无丢弃有界入队边界。
#include "slope_sim/client/recorder_queue.hpp"

#include <stdexcept>
#include <utility>

namespace slope_sim::client::v2 {

RecorderQueue::RecorderQueue(std::size_t capacity) : capacity_(capacity) {
  if (capacity_ == 0) {
    throw std::invalid_argument("recorder queue capacity must be positive");
  }
}

RecorderEnqueueResult RecorderQueue::Enqueue(RecordedRawFrame frame) {
  if (frames_.size() == capacity_) {
    return RecorderEnqueueResult::kOverflow;
  }
  frames_.push_back(std::move(frame));
  return RecorderEnqueueResult::kAccepted;
}

RecordedRawFrame RecorderQueue::Pop() {
  if (frames_.empty()) {
    throw std::runtime_error("recorder queue is empty");
  }
  RecordedRawFrame frame = std::move(frames_.front());
  frames_.pop_front();
  return frame;
}

std::size_t RecorderQueue::size() const noexcept {
  return frames_.size();
}

}  // namespace slope_sim::client::v2
