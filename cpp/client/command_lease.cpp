// 阶段四 C1：实现固定 100 ms 命令租约，超时后永远返回同形状零命令。
#include "slope_sim/client/command_lease.hpp"

#include <cmath>
#include <stdexcept>

namespace slope_sim::client::v2 {

WheelCommandLease::WheelCommandLease(
    std::size_t drive_count,
    std::size_t steering_count)
    : zero_motion_{std::vector<float>(drive_count, 0.0F),
                   std::vector<float>(steering_count, 0.0F)},
      latest_motion_(zero_motion_) {
  if (drive_count == 0) {
    throw std::invalid_argument("drive_count must be positive");
  }
}

void WheelCommandLease::Renew(WheelMotion command, std::chrono::milliseconds now) {
  ValidateTime(now);
  ValidateMotion(command);
  latest_motion_ = std::move(command);
  last_renewed_at_ = now;
  last_observed_at_ = now;
  has_observed_time_ = true;
  has_renewal_ = true;
  state_ = CommandLeaseState::kActive;
}

WheelMotion WheelCommandLease::Decision(std::chrono::milliseconds now) {
  ValidateTime(now);
  last_observed_at_ = now;
  has_observed_time_ = true;
  if (!has_renewal_) {
    return zero_motion_;
  }
  // 与 Python mailbox 一致：达到 100 ms 的边界立即停车，而非等待下一轮。
  if (now - last_renewed_at_ >= kTimeout) {
    state_ = CommandLeaseState::kTimedOut;
    return zero_motion_;
  }
  state_ = CommandLeaseState::kActive;
  return latest_motion_;
}

void WheelCommandLease::ValidateTime(std::chrono::milliseconds now) const {
  if (now.count() < 0) {
    throw std::invalid_argument("command lease time must be nonnegative");
  }
  if (has_observed_time_ && now < last_observed_at_) {
    throw std::invalid_argument("command lease time must not move backwards");
  }
}

void WheelCommandLease::ValidateMotion(const WheelMotion& command) const {
  if (command.drive_wheel_speed_rad_s.size() != zero_motion_.drive_wheel_speed_rad_s.size() ||
      command.steering_wheel_speed_rad_s.size() != zero_motion_.steering_wheel_speed_rad_s.size()) {
    throw std::invalid_argument("command wheel count does not match lease");
  }
  const auto require_finite = [](const std::vector<float>& values) {
    for (const float value : values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("command wheel speed must be finite");
      }
    }
  };
  require_finite(command.drive_wheel_speed_rad_s);
  require_finite(command.steering_wheel_speed_rad_s);
}

}  // namespace slope_sim::client::v2
