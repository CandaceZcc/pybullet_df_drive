// 阶段四 C1：SDK 对冻结五 topic 原始 protobuf payload 的统一验证入口。
#pragma once

#include <string_view>

namespace slope_sim::client::v2 {

/// 原始 v2 payload 的可观测拒绝类别，供所有 C++ consumer 共享。
enum class RawV2PayloadValidation {
  kValid,
  kUnknownTopic,
  kInvalidPayload,
  kDescriptorDigestMismatch,
};

/// 按冻结 topic 的顶层 protobuf 类型解析 payload，并验证内嵌 descriptor SHA-256。
RawV2PayloadValidation ValidateRawV2Payload(
    std::string_view topic,
    std::string_view payload,
    std::string_view expected_descriptor_sha256);

}  // namespace slope_sim::client::v2
