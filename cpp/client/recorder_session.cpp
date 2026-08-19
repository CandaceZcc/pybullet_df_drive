// 阶段四 C2：把无阻塞入队和可能失败的 MCAP 持久化置于不同调用边界。
#include "slope_sim/client/recorder_session.hpp"

#include <stdexcept>
#include <utility>

namespace slope_sim::client::v2 {

RecorderSession::RecorderSession(std::size_t queue_capacity,
                                 std::filesystem::path final_path,
                                 std::vector<std::byte> descriptor_set,
                                 McapSessionIdentity identity)
    : queue_(queue_capacity), writer_(std::move(final_path), std::move(descriptor_set), std::move(identity)) {}

RecorderEnqueueResult RecorderSession::Enqueue(RecordedRawFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != RecorderSessionState::kRecording || finalizing_) {
    return RecorderEnqueueResult::kFaulted;
  }
  const RecorderEnqueueResult result = queue_.Enqueue(std::move(frame));
  if (result == RecorderEnqueueResult::kOverflow) {
    // 队列满意味着已无法保证无丢弃记录，立即将控制面切换到安全停车状态。
    state_ = RecorderSessionState::kSafeStopRequired;
  }
  return result;
}

bool RecorderSession::DrainOne() {
  RecordedRawFrame frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != RecorderSessionState::kRecording || finalizing_ || queue_.size() == 0) {
      return false;
    }
    frame = queue_.Pop();
  }
  try {
    writer_.Write(frame.topic, frame.sequence, frame.log_time_ns, frame.publish_time_ns, frame.payload);
    return true;
  } catch (const std::exception&) {
    // 持久化失败不能回流到物理 callback；consumer 仅留下可轮询的安全停车状态。
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = RecorderSessionState::kSafeStopRequired;
    return false;
  }
}

bool RecorderSession::Finalize() {
  while (DrainOne()) {
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != RecorderSessionState::kRecording || queue_.size() != 0) {
      return false;
    }
    finalizing_ = true;
  }
  try {
    writer_.Finalize();
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = RecorderSessionState::kFinalized;
    return true;
  } catch (const std::exception&) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = RecorderSessionState::kSafeStopRequired;
    return false;
  }
}

RecorderSessionState RecorderSession::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace slope_sim::client::v2
