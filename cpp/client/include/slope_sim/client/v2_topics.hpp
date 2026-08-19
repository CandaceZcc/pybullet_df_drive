// 阶段四 C1：C++ SDK 对外暴露的冻结 v2 五话题元数据。
#pragma once

#include <array>

namespace slope_sim::client::v2 {

/// eCAL topic 在单机运行时中的单向角色。
enum class Direction {
  kPublish,
  kSubscribe,
};

/// 一条冻结 v2 线协议的名称、类型、频率和角色。
struct TopicContract final {
  const char* topic;
  const char* type_name;
  int rate_hz;
  Direction direction;
};

/// 返回 SDK 唯一的五 topic 合同，供后续 Command、Subscriber 与 Recorder 复用。
const std::array<TopicContract, 5>& TopicContracts();

}  // namespace slope_sim::client::v2
