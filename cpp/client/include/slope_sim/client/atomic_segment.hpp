// 阶段四 C2：Recorder segment 的临时写入与原子发布边界。
#pragma once

#include <filesystem>
#include <vector>

namespace slope_sim::client::v2 {

class AtomicSegment final {
 public:
  explicit AtomicSegment(std::filesystem::path final_path);
  ~AtomicSegment();

  AtomicSegment(const AtomicSegment&) = delete;
  AtomicSegment& operator=(const AtomicSegment&) = delete;

  void Append(const std::vector<std::byte>& bytes);
  void Finalize();

 private:
  std::filesystem::path final_path_;
  std::filesystem::path temporary_path_;
  int file_descriptor_ = -1;
  bool finalized_ = false;
};

}  // namespace slope_sim::client::v2
