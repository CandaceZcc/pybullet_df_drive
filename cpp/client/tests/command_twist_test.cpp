// runSim v2：线/角速度必须按车型生成完整且有界的轮子命令形状。
#include "slope_sim/client/command_twist.hpp"

#include <cmath>
#include <stdexcept>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
  using slope_sim::client::v2::MotionForTwist;
  using slope_sim::client::v2::RobotCommandShape;

  const auto differential = MotionForTwist(RobotCommandShape::kDifferential, 0.4F, 0.2F);
  Require(differential.drive_wheel_speed_rad_s.size() == 2U, "differential drive shape differs");
  Require(differential.steering_wheel_speed_rad_s.empty(), "differential steering shape differs");
  Require(std::abs(differential.drive_wheel_speed_rad_s[0] - 3.5F) < 1e-6F, "left speed differs");
  Require(std::abs(differential.drive_wheel_speed_rad_s[1] - 4.5F) < 1e-6F, "right speed differs");

  const auto active = MotionForTwist(RobotCommandShape::kActiveSteering4wd, 0.4F, 0.2F);
  Require(active.drive_wheel_speed_rad_s.size() == 4U, "active drive shape differs");
  Require(active.steering_wheel_speed_rad_s.size() == 2U, "active steering shape differs");
  for (const float speed : active.drive_wheel_speed_rad_s) {
    Require(std::abs(speed - 4.0F) < 1e-6F, "active drive speed differs");
  }
  Require(active.steering_wheel_speed_rad_s[0] > 0.0F, "active steering is not positive");
  Require(active.steering_wheel_speed_rad_s[0] == active.steering_wheel_speed_rad_s[1],
          "active steering speeds differ");

  bool rejected = false;
  try {
    (void)MotionForTwist(RobotCommandShape::kDifferential, 1.21F, 0.0F);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Require(rejected, "out-of-range twist was accepted");
  return 0;
}
