// 阶段四 C2：将冻结 v2 原始帧写入官方 MCAP，并在会话完成后原子发布。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace slope_sim::client::v2 {

inline constexpr std::string_view kMid360PatternVersion = "livox-mid360-800000-v1";
inline constexpr std::array<std::byte, 32> kMid360PatternSha256{
    std::byte{0x40}, std::byte{0x77}, std::byte{0xe0}, std::byte{0xb6},
    std::byte{0x8a}, std::byte{0x68}, std::byte{0xe4}, std::byte{0x0b},
    std::byte{0xa8}, std::byte{0xa5}, std::byte{0xda}, std::byte{0x17},
    std::byte{0xd4}, std::byte{0xaf}, std::byte{0xf5}, std::byte{0xba},
    std::byte{0x86}, std::byte{0xea}, std::byte{0x4f}, std::byte{0xb5},
    std::byte{0x57}, std::byte{0xa4}, std::byte{0xf8}, std::byte{0xb5},
    std::byte{0x94}, std::byte{0xe4}, std::byte{0xde}, std::byte{0x1e},
    std::byte{0xbb}, std::byte{0xeb}, std::byte{0x20}, std::byte{0xca},
};

/// 录制文件必须绑定到唯一仿真会话、冻结 descriptor、world、场景与扫描点阵身份。
struct McapSessionIdentity final {
  std::array<std::byte, 16> simulation_session_id;
  std::array<std::byte, 32> descriptor_sha256;
  std::uint64_t world_generation;
  std::string scene_id;
  std::string lidar_pattern_version;
  std::array<std::byte, 32> lidar_pattern_sha256;
};

class McapSessionWriter final {
 public:
  McapSessionWriter(std::filesystem::path final_path,
                    std::vector<std::byte> descriptor_set,
                    McapSessionIdentity identity);
  ~McapSessionWriter();

  McapSessionWriter(const McapSessionWriter&) = delete;
  McapSessionWriter& operator=(const McapSessionWriter&) = delete;

  /// 写入已冻结的原始 protobuf bytes，不在 Recorder 中重新编码消息。
  void Write(std::string_view topic,
             std::uint32_t sequence,
             std::uint64_t log_time_ns,
             std::uint64_t publish_time_ns,
             const std::vector<std::byte>& payload);
  /// 写入 MCAP footer/summary 后，以 AtomicSegment 的 fsync + rename 发布文件。
  void Finalize();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slope_sim::client::v2
