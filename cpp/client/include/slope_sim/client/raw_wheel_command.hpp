// 阶段四 C1：SDK 对 WheelCommand 原始 bytes 的唯一 protobuf/digest 判据。
#pragma once

#include <string_view>

namespace slope_sim::client::v2 {

/// 原始 WheelCommand 的可观测拒绝类别，供 Command 和 Subscriber 统一处理。
enum class RawWheelCommandValidation {
  kValid,
  kInvalidPayload,
  kDescriptorDigestMismatch,
};

/// 解析原始 WheelCommand，并验证其中的 descriptor SHA-256 与期望值逐 byte 相等。
RawWheelCommandValidation ValidateRawWheelCommand(
    std::string_view payload,
    std::string_view expected_descriptor_sha256);

}  // namespace slope_sim::client::v2
