// 阶段四 C2：Recorder 写入线程前的有界原始帧队列。
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace slope_sim::client::v2 {

/// 一条待持久化的冻结 topic 原始 bytes，Recorder 不在队列中重新编码。
struct RecordedRawFrame final {
  std::string topic;
  std::vector<std::byte> payload;
  std::uint32_t sequence = 0;
  std::uint64_t log_time_ns = 0;
  std::uint64_t publish_time_ns = 0;
};

/// 队列满或已故障必须显式报告，调用方据此触发安全停止而不是静默丢帧。
enum class RecorderEnqueueResult { kAccepted, kOverflow, kFaulted };

class RecorderQueue final {
 public:
  explicit RecorderQueue(std::size_t capacity);

  RecorderEnqueueResult Enqueue(RecordedRawFrame frame);
  RecordedRawFrame Pop();
  [[nodiscard]] std::size_t size() const noexcept;

 private:
  std::size_t capacity_;
  std::deque<RecordedRawFrame> frames_;
};

}  // namespace slope_sim::client::v2
