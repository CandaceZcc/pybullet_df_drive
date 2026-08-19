// 阶段四 C1：复用生成 protobuf 对 WheelCommand 原始 payload 做唯一判据。
#include "slope_sim/client/raw_wheel_command.hpp"

#include "slope_sim/client/raw_v2_payload.hpp"

namespace slope_sim::client::v2 {

RawWheelCommandValidation ValidateRawWheelCommand(
    std::string_view payload,
    std::string_view expected_descriptor_sha256) {
  // 兼容旧 Command 调用面，但所有 WheelCommand 语义统一由五 topic 判据裁决。
  const auto result = ValidateRawV2Payload(
      "/sim/wheel/command", payload, expected_descriptor_sha256);
  if (result == RawV2PayloadValidation::kDescriptorDigestMismatch) {
    return RawWheelCommandValidation::kDescriptorDigestMismatch;
  }
  return result == RawV2PayloadValidation::kValid
      ? RawWheelCommandValidation::kValid
      : RawWheelCommandValidation::kInvalidPayload;
}

}  // namespace slope_sim::client::v2
