// 阶段四 D：ROS Bridge 发布 TF/点云前必须验证同一 v2 传感器快照身份。
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace slope_sim::client::v2 {

/// 一组可安全投影给 ROS 的 LiDAR、RTK、IMU 共享 identity 与时间戳。
struct RosSensorPair final {
  std::array<std::byte, 16> simulation_session_id;
  std::uint64_t world_generation;
  std::uint64_t timestamp_ns;
};

/// 拒绝任意不同 session/generation/timestamp 的三类传感器原始 v2 payload。
RosSensorPair ValidateRosSensorPair(
    std::string_view lidar_payload,
    std::string_view rtk_payload,
    std::string_view imu_payload,
    std::string_view expected_descriptor_sha256);

}  // namespace slope_sim::client::v2
