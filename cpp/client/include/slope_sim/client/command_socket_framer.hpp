// runSim v2：交互 Unix SOCK_STREAM 的有界 NDJSON 帧累积器。
#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace slope_sim::client::v2 {

class CommandSocketFramer final {
 public:
  static constexpr std::size_t kMaximumFrameBytes = 1024U;

  enum class Result { kIncomplete, kFrame, kOversize };

  /// LF 是帧的一部分；因此可承载的 JSON 文本最多 1023 bytes。
  Result Append(std::string_view bytes, std::vector<std::string>* frames) {
    inbound_.append(bytes.data(), bytes.size());
    bool emitted = false;
    while (true) {
      const std::size_t newline = inbound_.find('\n');
      if (newline == std::string::npos) {
        if (inbound_.size() >= kMaximumFrameBytes) {
          inbound_.clear();
          return Result::kOversize;
        }
        return emitted ? Result::kFrame : Result::kIncomplete;
      }
      if (newline + 1U > kMaximumFrameBytes) {
        inbound_.clear();
        return Result::kOversize;
      }
      frames->push_back(inbound_.substr(0, newline));
      inbound_.erase(0, newline + 1U);
      emitted = true;
    }
  }

  void Clear() { inbound_.clear(); }

 private:
  std::string inbound_;
};

/// 认证 stop 是本批的终止边界，后续已合帧的控制消息不得再改变命令状态。
template <typename ApplyFrame>
bool ProcessCommandSocketFramesUntilTerminal(
    const std::vector<std::string>& frames,
    ApplyFrame&& apply_frame) {
  for (const std::string& frame : frames) {
    if (apply_frame(frame)) return true;
  }
  return false;
}

}  // namespace slope_sim::client::v2
