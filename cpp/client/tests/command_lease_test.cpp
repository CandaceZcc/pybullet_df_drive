// 阶段四 C1：命令租约在精确超时边界输出同车型零命令。
#include "slope_sim/client/command_lease.hpp"

#include <cassert>
#include <chrono>
#include <vector>

int main() {
  using namespace std::chrono_literals;
  using slope_sim::client::v2::CommandLeaseState;
  using slope_sim::client::v2::WheelCommandLease;
  using slope_sim::client::v2::WheelMotion;

  WheelCommandLease lease(4, 2);
  assert(lease.state() == CommandLeaseState::kWaiting);
  assert((lease.Decision(0ms) == WheelMotion{{0.0F, 0.0F, 0.0F, 0.0F}, {0.0F, 0.0F}}));

  const WheelMotion command{{1.0F, -2.0F, 3.0F, -4.0F}, {0.25F, -0.5F}};
  lease.Renew(command, 10ms);
  assert(lease.state() == CommandLeaseState::kActive);
  assert(lease.Decision(109ms) == command);
  assert(lease.state() == CommandLeaseState::kActive);

  assert((lease.Decision(110ms) == WheelMotion{{0.0F, 0.0F, 0.0F, 0.0F}, {0.0F, 0.0F}}));
  assert(lease.state() == CommandLeaseState::kTimedOut);
  return 0;
}
