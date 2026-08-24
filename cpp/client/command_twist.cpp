// runSim v2：C++ Command 的最小车型运动学，不接受 Dashboard 传入轮速或协议身份。
#include "slope_sim/client/command_twist.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace slope_sim::client::v2 {
namespace {

// 与 runsim_session.py 的正式 socket 合同一致，避免合法 GUI/RC 目标被 Command 静默归零。
constexpr float kMaxLinearVelocityMps = 3.0F;
constexpr float kMaxAngularVelocityRadS = 1.2F;
constexpr float kWheelRadiusM = 0.10F;
constexpr float kWheelTrackM = 0.50F;
constexpr float kAxleDistanceM = 0.52F;
constexpr float kMaxSteeringAngleRad = 0.55F;
constexpr float kMaxSteeringSpeedRadS = 2.0F;
constexpr float kCommandPeriodSec = 0.01F;

void ValidateTwist(float linear_velocity_m_s, float angular_velocity_rad_s) {
  if (!std::isfinite(linear_velocity_m_s) || !std::isfinite(angular_velocity_rad_s) ||
      std::abs(linear_velocity_m_s) > kMaxLinearVelocityMps ||
      std::abs(angular_velocity_rad_s) > kMaxAngularVelocityRadS) {
    throw std::invalid_argument("interactive target velocity is out of range");
  }
}

}  // namespace

WheelMotion MotionForTwist(
    RobotCommandShape shape,
    float linear_velocity_m_s,
    float angular_velocity_rad_s,
    float steering_angle_rad) {
  ValidateTwist(linear_velocity_m_s, angular_velocity_rad_s);
  if (shape == RobotCommandShape::kDifferential) {
    const float left = (linear_velocity_m_s - angular_velocity_rad_s * kWheelTrackM / 2.0F) /
        kWheelRadiusM;
    const float right = (linear_velocity_m_s + angular_velocity_rad_s * kWheelTrackM / 2.0F) /
        kWheelRadiusM;
    return {{left, right}, {}};
  }

  const float desired_angle = std::abs(linear_velocity_m_s) < 1e-6F
      ? 0.0F
      : std::clamp(
          std::atan(kAxleDistanceM * angular_velocity_rad_s / linear_velocity_m_s),
          -kMaxSteeringAngleRad,
          kMaxSteeringAngleRad);
  const float steering_speed = std::clamp(
      (desired_angle - steering_angle_rad) / kCommandPeriodSec,
      -kMaxSteeringSpeedRadS,
      kMaxSteeringSpeedRadS);
  const float drive_speed = linear_velocity_m_s / kWheelRadiusM;
  return {{drive_speed, drive_speed, drive_speed, drive_speed}, {steering_speed, steering_speed}};
}

WheelMotion TwistCommandConverter::Convert(float linear_velocity_m_s, float angular_velocity_rad_s) {
  const WheelMotion motion = MotionForTwist(
      shape_, linear_velocity_m_s, angular_velocity_rad_s, steering_angle_rad_);
  if (shape_ == RobotCommandShape::kActiveSteering4wd) {
    steering_angle_rad_ = std::clamp(
        steering_angle_rad_ + motion.steering_wheel_speed_rad_s.front() * kCommandPeriodSec,
        -kMaxSteeringAngleRad,
        kMaxSteeringAngleRad);
  }
  return motion;
}

}  // namespace slope_sim::client::v2
