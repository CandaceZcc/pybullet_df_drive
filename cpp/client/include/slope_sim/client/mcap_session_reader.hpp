// 阶段四 D：下游 Replay/Export 只接受已完成且身份完整的原始 MCAP 会话。
#pragma once

#include <filesystem>
#include <vector>

#include "slope_sim/client/mcap_session_writer.hpp"
#include "slope_sim/client/recorder_queue.hpp"

namespace slope_sim::client::v2 {

/// 验证已完成 MCAP 的冻结 schema、五 topic 与会话 identity，并返回只读身份摘要。
McapSessionIdentity VerifyCompletedMcapSession(const std::filesystem::path& path);

/// 已验证会话的 identity 与原始帧；Replay/Export 只能消费此只读结果。
struct CompletedMcapSession final {
  McapSessionIdentity identity;
  std::vector<RecordedRawFrame> frames;
};

/// 验证完成状态后，按 MCAP 文件顺序读取原始帧，不重新编码 payload。
CompletedMcapSession ReadCompletedMcapSession(const std::filesystem::path& path);

}  // namespace slope_sim::client::v2
