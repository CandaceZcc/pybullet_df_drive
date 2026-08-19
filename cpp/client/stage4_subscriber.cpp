// 阶段四 C1：只读 C++ eCAL Subscriber，验证冻结五 topic 的 raw metadata 与完整窗口。
#include <charconv>
#include <atomic>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include <ecal/ecal.h>

#include "../common/sha256.hpp"
#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace {

namespace fs = std::filesystem;

std::string ReadInput(const std::string& raw_path) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("input must be an absolute normalized regular file");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("input cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int PositiveInteger(const std::string& raw, const char* name) {
  int value = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), value);
  if (error != std::errc{} || end != raw.data() + raw.size() || value < 1 || value > 60000) {
    throw std::invalid_argument(std::string(name) + " must be an integer in [1, 60000]");
  }
  return value;
}

std::map<std::string, std::string> ParseOptions(int argc, char* argv[]) {
  std::map<std::string, std::string> options;
  for (int index = 1; index < argc; index += 2) {
    if (index + 1 >= argc || std::string(argv[index]).rfind("--", 0) != 0 ||
        !options.emplace(argv[index], argv[index + 1]).second) {
      throw std::invalid_argument("options must be unique key/value pairs");
    }
  }
  return options;
}

const slope_sim::client::v2::TopicContract* FindTopicContract(const std::string& topic) {
  for (const auto& contract : slope_sim::client::v2::TopicContracts()) {
    if (topic == contract.topic) {
      return &contract;
    }
  }
  return nullptr;
}

eCAL::SDataTypeInformation TopicTypeInfo(
    const slope_sim::client::v2::TopicContract& contract,
    const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = contract.type_name;
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

void WriteNewResult(const std::string& raw_path, const std::string& topic, int received_count) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path())) {
    throw std::invalid_argument("result must have an existing absolute parent directory");
  }
  const int descriptor = open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  if (descriptor < 0) throw std::runtime_error("result must not already exist");
  const std::string document = "{\"clean_shutdown\":true,\"received_count\":" +
      std::to_string(received_count) + ",\"role\":\"subscriber\",\"topic\":\"" +
      topic + "\"}\n";
  const ssize_t written = write(descriptor, document.data(), document.size());
  const bool complete = written == static_cast<ssize_t>(document.size());
  (void)close(descriptor);
  if (!complete) throw std::runtime_error("subscriber result write failed");
}

int RunSingle(const std::map<std::string, std::string>& options) {
  if (options.size() != 5 || options.find("--topic") == options.end() ||
      options.find("--descriptor-set") == options.end() || options.find("--expected-count") == options.end() ||
      options.find("--deadline-ms") == options.end() || options.find("--result") == options.end()) {
    throw std::invalid_argument("subscriber options are incomplete");
  }
  const std::string topic = options.at("--topic");
  const auto* contract = FindTopicContract(topic);
  if (contract == nullptr) {
    throw std::invalid_argument("subscriber topic is not a frozen v2 contract");
  }
  const std::string descriptor = ReadInput(options.at("--descriptor-set"));
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  const int expected_count = PositiveInteger(options.at("--expected-count"), "expected-count");
  const int deadline_ms = PositiveInteger(options.at("--deadline-ms"), "deadline-ms");
  std::atomic<int> received_count{0};
  std::atomic<int> rejected_count{0};
  std::atomic<bool> observed_single_publisher{false};
  std::atomic<bool> observed_publisher_conflict{false};
  if (!eCAL::Initialize("slope-sim-stage4-subscriber")) {
    throw std::runtime_error("eCAL initialization failed");
  }
  bool initialized = true;
  try {
    {
      eCAL::CSubscriber subscriber(topic, TopicTypeInfo(*contract, descriptor));
      subscriber.SetReceiveCallback(
          [&digest, &topic, &contract, &received_count, &rejected_count](
              const eCAL::STopicId&,
              const eCAL::SDataTypeInformation& type_info,
              const eCAL::SReceiveCallbackData& data) {
            if (type_info.name != contract->type_name || type_info.encoding != "proto" ||
                stage4::Bytes(stage4::Sha256(type_info.descriptor)) != digest ||
                (data.buffer_size > 0 && data.buffer == nullptr)) {
              ++rejected_count;
              return;
            }
            const std::string_view payload(static_cast<const char*>(data.buffer), data.buffer_size);
            if (slope_sim::client::v2::ValidateRawV2Payload(topic, payload, digest) !=
                slope_sim::client::v2::RawV2PayloadValidation::kValid) {
              ++rejected_count;
              return;
            }
            ++received_count;
          });
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
      while (std::chrono::steady_clock::now() < deadline && received_count.load() < expected_count &&
             rejected_count.load() == 0) {
        const std::size_t publisher_count = subscriber.GetPublisherCount();
        if (publisher_count == 1) {
          observed_single_publisher = true;
        } else if (publisher_count > 1) {
          // 窗口内曾竞争过的 producer 不能由随后退出的 publisher 掩盖。
          observed_publisher_conflict = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      if (!observed_single_publisher.load() || observed_publisher_conflict.load() ||
          received_count.load() != expected_count ||
          rejected_count.load() != 0) {
        throw std::runtime_error(
            "subscriber did not receive the exact verified v2 window: publishers=" +
            std::to_string(subscriber.GetPublisherCount()) + ", received=" +
            std::to_string(received_count.load()) + ", rejected=" +
            std::to_string(rejected_count.load()) + ", conflict=" +
            std::to_string(observed_publisher_conflict.load()));
      }
    }
    eCAL::Finalize();
    initialized = false;
    WriteNewResult(options.at("--result"), topic, received_count.load());
    return 0;
  } catch (...) {
    if (initialized) eCAL::Finalize();
    throw;
  }
}

struct OutputReceipt final {
  const slope_sim::client::v2::TopicContract* contract = nullptr;
  int expected_count = 0;
  std::atomic<int> received_count{0};
  std::atomic<int> rejected_count{0};
  std::atomic<bool> observed_single_publisher{false};
  std::atomic<bool> observed_publisher_conflict{false};
};

int OutputDurationMs(const std::string& raw) {
  const int duration_ms = PositiveInteger(raw, "duration-ms");
  if (duration_ms < 100 || duration_ms % 100 != 0) {
    throw std::invalid_argument("duration-ms must be a 100 ms multiple in [100, 60000]");
  }
  return duration_ms;
}

void WriteAllOutputsResult(const std::string& raw_path, const std::array<OutputReceipt, 4>& receipts) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path())) {
    throw std::invalid_argument("result must have an existing absolute parent directory");
  }
  const int descriptor = open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  if (descriptor < 0) throw std::runtime_error("result must not already exist");
  std::string topics;
  for (std::size_t index = 0; index < receipts.size(); ++index) {
    if (index != 0) topics += ',';
    topics += "\"" + std::string(receipts[index].contract->topic) + "\":" +
        std::to_string(receipts[index].received_count.load());
  }
  const std::string document = "{\"clean_shutdown\":true,\"role\":\"subscriber\",\"topics\":{" +
      topics + "}}\n";
  const ssize_t written = write(descriptor, document.data(), document.size());
  const bool complete = written == static_cast<ssize_t>(document.size());
  (void)close(descriptor);
  if (!complete) throw std::runtime_error("subscriber result write failed");
}

int RunAllOutputs(const std::map<std::string, std::string>& options) {
  if (options.size() != 5 || options.at("--all-outputs") != "true" ||
      options.find("--descriptor-set") == options.end() || options.find("--duration-ms") == options.end() ||
      options.find("--deadline-ms") == options.end() || options.find("--result") == options.end()) {
    throw std::invalid_argument("all-output subscriber options are incomplete");
  }
  const std::string descriptor = ReadInput(options.at("--descriptor-set"));
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  const int duration_ms = OutputDurationMs(options.at("--duration-ms"));
  const int deadline_ms = PositiveInteger(options.at("--deadline-ms"), "deadline-ms");
  const auto& contracts = slope_sim::client::v2::TopicContracts();
  std::array<OutputReceipt, 4> receipts{};
  for (std::size_t index = 0; index < receipts.size(); ++index) {
    const auto& contract = contracts[index + 1];
    if (contract.direction != slope_sim::client::v2::Direction::kPublish) {
      throw std::runtime_error("frozen output topic has an invalid direction");
    }
    receipts[index].contract = &contract;
    receipts[index].expected_count = contract.rate_hz * duration_ms / 1000;
  }
  if (!eCAL::Initialize("slope-sim-stage4-subscriber")) {
    throw std::runtime_error("eCAL initialization failed");
  }
  bool initialized = true;
  try {
    {
      std::array<std::unique_ptr<eCAL::CSubscriber>, 4> subscribers;
      for (std::size_t index = 0; index < receipts.size(); ++index) {
        OutputReceipt* const receipt = &receipts[index];
        subscribers[index] = std::make_unique<eCAL::CSubscriber>(
            receipt->contract->topic, TopicTypeInfo(*receipt->contract, descriptor));
        subscribers[index]->SetReceiveCallback(
            [&digest, receipt](
                const eCAL::STopicId&,
                const eCAL::SDataTypeInformation& type_info,
                const eCAL::SReceiveCallbackData& data) {
              if (type_info.name != receipt->contract->type_name || type_info.encoding != "proto" ||
                  stage4::Bytes(stage4::Sha256(type_info.descriptor)) != digest ||
                  (data.buffer_size > 0 && data.buffer == nullptr)) {
                ++receipt->rejected_count;
                return;
              }
              const std::string_view payload(static_cast<const char*>(data.buffer), data.buffer_size);
              if (slope_sim::client::v2::ValidateRawV2Payload(receipt->contract->topic, payload, digest) !=
                  slope_sim::client::v2::RawV2PayloadValidation::kValid) {
                ++receipt->rejected_count;
                return;
              }
              ++receipt->received_count;
            });
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
      while (std::chrono::steady_clock::now() < deadline) {
        bool complete = true;
        for (std::size_t index = 0; index < receipts.size(); ++index) {
          auto& receipt = receipts[index];
          const std::size_t publisher_count = subscribers[index]->GetPublisherCount();
          if (publisher_count == 1) {
            receipt.observed_single_publisher = true;
          } else if (publisher_count > 1) {
            // 每条 output 都必须在完整窗口内保持唯一 Python runtime producer。
            receipt.observed_publisher_conflict = true;
          }
          complete = complete && receipt.received_count.load() >= receipt.expected_count &&
              receipt.rejected_count.load() == 0;
        }
        if (complete) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      for (std::size_t index = 0; index < receipts.size(); ++index) {
        if (!receipts[index].observed_single_publisher.load() ||
            receipts[index].observed_publisher_conflict.load() ||
            receipts[index].received_count.load() != receipts[index].expected_count ||
            receipts[index].rejected_count.load() != 0) {
          throw std::runtime_error("all-output subscriber did not receive the exact verified v2 window");
        }
      }
    }
    eCAL::Finalize();
    initialized = false;
    WriteAllOutputsResult(options.at("--result"), receipts);
    return 0;
  } catch (...) {
    if (initialized) eCAL::Finalize();
    throw;
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto options = ParseOptions(argc, argv);
    return options.find("--all-outputs") != options.end() ? RunAllOutputs(options) : RunSingle(options);
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
