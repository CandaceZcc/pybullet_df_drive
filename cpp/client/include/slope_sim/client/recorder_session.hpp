// 阶段四 C2：将实时 callback 的入队与独立 Recorder consumer 的持久化隔离。
#pragma once

#include <cstddef>
#include <filesystem>
#include <mutex>
#include <vector>

#include "slope_sim/client/mcap_session_writer.hpp"
#include "slope_sim/client/recorder_queue.hpp"

namespace slope_sim::client::v2 {

/// Recorder 失败后必须保持安全停车请求，禁止继续写入一个不完整会话。
enum class RecorderSessionState {
  kRecording,
  kSafeStopRequired,
  kFinalized,
};

class RecorderSession final {
 public:
  RecorderSession(std::size_t queue_capacity,
                  std::filesystem::path final_path,
                  std::vector<std::byte> descriptor_set,
                  McapSessionIdentity identity);

  /// 供实时 callback 调用：只移动内存，不做磁盘 IO。
  RecorderEnqueueResult Enqueue(RecordedRawFrame frame);
  /// 供 Recorder consumer 调用：至多持久化一帧，失败后返回 false 并请求安全停车。
  bool DrainOne();
  /// 停止阶段排空队列并发布 MCAP；失败时返回 false，调用方应安全停车。
  bool Finalize();
  [[nodiscard]] RecorderSessionState state() const noexcept;

 private:
  mutable std::mutex mutex_;
  RecorderQueue queue_;
  McapSessionWriter writer_;
  RecorderSessionState state_ = RecorderSessionState::kRecording;
  bool finalizing_ = false;
};

}  // namespace slope_sim::client::v2
