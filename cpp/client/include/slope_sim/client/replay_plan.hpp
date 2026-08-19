// 阶段四 D：把已验证 Recorder 帧映射到隔离的 Replay topic，默认禁止轮控回放。
#pragma once

#include "slope_sim/client/recorder_queue.hpp"

namespace slope_sim::client::v2 {

/// 仅接受正式 Simulator 输出，并保持原始帧 bytes/时序不变地映射到 /replay/sim/*。
RecordedRawFrame PlanReplayFrame(const RecordedRawFrame& frame);

}  // namespace slope_sim::client::v2
