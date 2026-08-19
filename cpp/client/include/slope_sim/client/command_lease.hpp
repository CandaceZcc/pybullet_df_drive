// 阶段四 C1：C++ Command 的固定 100 ms 命令租约与安全停止判据。
#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

namespace slope_sim::client::v2 {

/// 一帧按车型轮组形状排列的命令值。
struct WheelMotion final {
  std::vector<float> drive_wheel_speed_rad_s;
  std::vector<float> steering_wheel_speed_rad_s;

  bool operator==(const WheelMotion& other) const {
    return drive_wheel_speed_rad_s == other.drive_wheel_speed_rad_s &&
        steering_wheel_speed_rad_s == other.steering_wheel_speed_rad_s;
  }
};

/// Command 当前可观测的租约状态。
enum class CommandLeaseState {
  kWaiting,
  kActive,
  kTimedOut,
};

/// 在固定 100 ms 边界安全停止的单线程命令租约。
class WheelCommandLease final {
 public:
  static constexpr std::chrono::milliseconds kTimeout{100};

  WheelCommandLease(std::size_t drive_count, std::size_t steering_count);

  /// 用同车型、有限轮速的命令续租；时间必须相对于此前操作非递减。
  void Renew(WheelMotion command, std::chrono::milliseconds now);

  /// 返回当前命令；达到租约边界即返回完整零数组并转换到超时状态。
  WheelMotion Decision(std::chrono::milliseconds now);

  CommandLeaseState state() const { return state_; }

 private:
  void ValidateTime(std::chrono::milliseconds now) const;
  void ValidateMotion(const WheelMotion& command) const;

  WheelMotion zero_motion_;
  WheelMotion latest_motion_;
  std::chrono::milliseconds last_observed_at_{0};
  std::chrono::milliseconds last_renewed_at_{0};
  bool has_observed_time_ = false;
  bool has_renewal_ = false;
  CommandLeaseState state_ = CommandLeaseState::kWaiting;
};

}  // namespace slope_sim::client::v2
