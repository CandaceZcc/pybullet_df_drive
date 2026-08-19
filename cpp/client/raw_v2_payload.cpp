// 阶段四 C1：以唯一 topic 合同选择对应 protobuf 顶层消息，避免 consumer 各自解析。
#include "slope_sim/client/raw_v2_payload.hpp"

#include <cstdint>
#include <limits>
#include <type_traits>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "slope_sim/client/v2_topics.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace slope_sim::client::v2 {
namespace {

bool HasUnknownFieldsRecursively(const google::protobuf::Message& message) {
  const auto* reflection = message.GetReflection();
  if (!reflection->GetUnknownFields(message).empty()) {
    return true;
  }
  const auto* descriptor = message.GetDescriptor();
  for (int index = 0; index < descriptor->field_count(); ++index) {
    const auto* field = descriptor->field(index);
    if (field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      continue;
    }
    if (field->is_repeated()) {
      const int size = reflection->FieldSize(message, field);
      for (int member_index = 0; member_index < size; ++member_index) {
        if (HasUnknownFieldsRecursively(reflection->GetRepeatedMessage(message, field, member_index))) {
          return true;
        }
      }
    } else if (reflection->HasField(message, field) &&
               HasUnknownFieldsRecursively(reflection->GetMessage(message, field))) {
      return true;
    }
  }
  return false;
}

template <typename Message>
RawV2PayloadValidation ValidateMessage(
    std::string_view payload,
    std::string_view expected_descriptor_sha256) {
  if (payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return RawV2PayloadValidation::kInvalidPayload;
  }
  Message message;
  if (!message.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
    return RawV2PayloadValidation::kInvalidPayload;
  }
  // 五个 v2 顶层消息都必须绑定到一个已建立的仿真 session/world。
  if (message.simulation_session_id().size() != 16 || message.world_generation() == 0) {
    return RawV2PayloadValidation::kInvalidPayload;
  }
  // Protobuf 默认保留未知字段；线协议拒绝任一嵌套层的未声明扩展。
  if (HasUnknownFieldsRecursively(message)) {
    return RawV2PayloadValidation::kInvalidPayload;
  }
  if constexpr (std::is_same_v<Message, slope_sim::interfaces::v2::LidarPointCloud>) {
    // 点数是 LiDAR 帧的结构合同，不能只信任 payload 的声明值。
    if (message.point_num() != static_cast<std::uint32_t>(message.points_size())) {
      return RawV2PayloadValidation::kInvalidPayload;
    }
  }
  if (message.descriptor_sha256() != expected_descriptor_sha256) {
    return RawV2PayloadValidation::kDescriptorDigestMismatch;
  }
  return RawV2PayloadValidation::kValid;
}

}  // namespace

RawV2PayloadValidation ValidateRawV2Payload(
    std::string_view topic,
    std::string_view payload,
    std::string_view expected_descriptor_sha256) {
  // topic 是线协议的类型判据；不允许由调用方任意指定 message 类型。
  const auto& contracts = TopicContracts();
  if (topic == contracts[0].topic) {
    return ValidateMessage<slope_sim::interfaces::v2::WheelCommand>(payload, expected_descriptor_sha256);
  }
  if (topic == contracts[1].topic) {
    return ValidateMessage<slope_sim::interfaces::v2::WheelState>(payload, expected_descriptor_sha256);
  }
  if (topic == contracts[2].topic) {
    return ValidateMessage<slope_sim::interfaces::v2::LidarPointCloud>(payload, expected_descriptor_sha256);
  }
  if (topic == contracts[3].topic) {
    return ValidateMessage<slope_sim::interfaces::v2::RtkState>(payload, expected_descriptor_sha256);
  }
  if (topic == contracts[4].topic) {
    return ValidateMessage<slope_sim::interfaces::v2::ImuAttitude>(payload, expected_descriptor_sha256);
  }
  return RawV2PayloadValidation::kUnknownTopic;
}

}  // namespace slope_sim::client::v2
