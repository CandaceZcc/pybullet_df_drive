// runSim v2：将 Dashboard 的有限 body twist 固定转换为当前车型的轮子速度形状。
#pragma once

#include "slope_sim/client/command_lease.hpp"

namespace slope_sim::client::v2 {

enum class RobotCommandShape {
  kDifferential,
  kActiveSteering4wd,
};

/// 以当前前轮估计角生成一帧轮子速度；只接受 UI 合同范围内的 v/w。
WheelMotion MotionForTwist(
    RobotCommandShape shape,
    float linear_velocity_m_s,
    float angular_velocity_rad_s,
    float steering_angle_rad = 0.0F);

/// Command 维护主动转向的本地角度估计，避免将目标角误写为永久速度。
class TwistCommandConverter final {
 public:
  explicit TwistCommandConverter(RobotCommandShape shape) : shape_(shape) {}

  WheelMotion Convert(float linear_velocity_m_s, float angular_velocity_rad_s);

 private:
  RobotCommandShape shape_;
  float steering_angle_rad_ = 0.0F;
};

}  // namespace slope_sim::client::v2
