// 阶段四 D：独立 eCAL Replay participant，只把已验证 MCAP 输出送到隔离 /replay/sim/*。
#include <array>
#include <charconv>
#include <chrono>
#include <cerrno>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <ecal/ecal.h>

#include "../common/sha256.hpp"
#include "slope_sim/client/mcap_session_reader.hpp"
#include "slope_sim/client/replay_plan.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace {

namespace fs = std::filesystem;
using slope_sim::client::v2::CompletedMcapSession;
using slope_sim::client::v2::RecordedRawFrame;
using slope_sim::client::v2::TopicContract;

struct ReplayPlan final {
  fs::path input;
  fs::path descriptor_set;
  fs::path result;
  int deadline_ms = 0;
};

struct ReplayFrame final {
  RecordedRawFrame source;
  RecordedRawFrame isolated;
  std::size_t output_index = 0;
};

/// 读取 CLI 的普通文件，并要求调用者已完成路径约束检查。
std::string ReadRegularFile(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("input cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

fs::path ExistingInputPath(const std::string& raw, const char* name) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument(std::string(name) + " must be an absolute normalized regular file");
  }
  return path;
}

fs::path NewResultPath(const std::string& raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path()) ||
      fs::exists(path)) {
    throw std::invalid_argument("result must be a new file below an existing absolute directory");
  }
  return path;
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
    if (index + 1 >= argc || std::string_view(argv[index]).rfind("--", 0) != 0 ||
        !options.emplace(argv[index], argv[index + 1]).second) {
      throw std::invalid_argument("options must be unique key/value pairs");
    }
  }
  return options;
}

ReplayPlan ParsePlan(int argc, char* argv[]) {
  const auto options = ParseOptions(argc, argv);
  if (options.size() != 4 || options.find("--input") == options.end() ||
      options.find("--descriptor-set") == options.end() || options.find("--deadline-ms") == options.end() ||
      options.find("--result") == options.end()) {
    throw std::invalid_argument("replay options are incomplete");
  }
  return {
      ExistingInputPath(options.at("--input"), "input"),
      ExistingInputPath(options.at("--descriptor-set"), "descriptor-set"),
      NewResultPath(options.at("--result")),
      PositiveInteger(options.at("--deadline-ms"), "deadline-ms"),
  };
}

eCAL::SDataTypeInformation TopicTypeInfo(const TopicContract& contract, const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = contract.type_name;
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

/// 等待所有隔离 publisher 都只连接一个 consumer；任何竞争者都是安全错误。
bool WaitForSingleSubscribers(
    const std::array<std::unique_ptr<eCAL::CPublisher>, 4>& publishers,
    int deadline_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_connected = true;
    for (const auto& publisher : publishers) {
      const std::size_t count = publisher->GetSubscriberCount();
      if (count > 1) throw std::runtime_error("replay publisher discovered competing subscribers");
      all_connected = all_connected && count == 1;
    }
    if (all_connected) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (const auto& publisher : publishers) {
    if (publisher->GetSubscriberCount() != 1) return false;
  }
  return true;
}

/// 成功结果以 O_EXCL 与 fsync 发布，避免 replay 覆盖先前审计结论。
void WriteNewResult(const fs::path& path, const std::array<int, 4>& counts) {
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) throw std::runtime_error("replay result cannot be created exclusively");
  const std::string document =
      "{\"clean_shutdown\":true,\"role\":\"replay\",\"topics\":{"
      "\"/replay/sim/wheel/state\":" + std::to_string(counts[0]) +
      ",\"/replay/sim/lidar/points\":" + std::to_string(counts[1]) +
      ",\"/replay/sim/rtk/state\":" + std::to_string(counts[2]) +
      ",\"/replay/sim/imu/attitude\":" + std::to_string(counts[3]) + "}}\n";
  std::size_t written = 0;
  while (written < document.size()) {
    const ssize_t count = ::write(descriptor, document.data() + written, document.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      (void)::close(descriptor);
      throw std::runtime_error("replay result write failed");
    }
    written += static_cast<std::size_t>(count);
  }
  if (::fsync(descriptor) != 0 || ::close(descriptor) != 0) {
    throw std::runtime_error("replay result sync failed");
  }
}

std::vector<ReplayFrame> PrepareReplay(
    const CompletedMcapSession& session,
    const std::string& descriptor) {
  const std::string expected_digest(
      reinterpret_cast<const char*>(session.identity.descriptor_sha256.data()),
      session.identity.descriptor_sha256.size());
  if (stage4::Bytes(stage4::Sha256(descriptor)) != expected_digest) {
    throw std::runtime_error("supplied descriptor differs from MCAP session identity");
  }

  const auto& contracts = slope_sim::client::v2::TopicContracts();
  std::vector<ReplayFrame> frames;
  frames.reserve(session.frames.size());
  for (const auto& source : session.frames) {
    const auto isolated = slope_sim::client::v2::PlanReplayFrame(source);
    std::size_t output_index = 0;
    bool found = false;
    for (std::size_t index = 1; index < contracts.size(); ++index) {
      if (source.topic == contracts[index].topic) {
        output_index = index - 1;
        found = true;
        break;
      }
    }
    if (!found) throw std::runtime_error("replay planner accepted a non-output frame");
    frames.push_back({source, isolated, output_index});
  }
  return frames;
}

int RunReplay(const ReplayPlan& plan) {
  const std::string descriptor = ReadRegularFile(plan.descriptor_set);
  const CompletedMcapSession session = slope_sim::client::v2::ReadCompletedMcapSession(plan.input);
  const std::vector<ReplayFrame> frames = PrepareReplay(session, descriptor);
  const auto& contracts = slope_sim::client::v2::TopicContracts();
  std::array<int, 4> published{};

  if (!eCAL::Initialize("slope-sim-stage4-replay")) {
    throw std::runtime_error("eCAL initialization failed");
  }
  bool initialized = true;
  try {
    {
      std::array<std::unique_ptr<eCAL::CPublisher>, 4> publishers;
      for (std::size_t index = 0; index < publishers.size(); ++index) {
        const auto& contract = contracts[index + 1];
        publishers[index] = std::make_unique<eCAL::CPublisher>(
            std::string("/replay") + contract.topic, TopicTypeInfo(contract, descriptor));
      }
      if (!WaitForSingleSubscribers(publishers, plan.deadline_ms)) {
        throw std::runtime_error("replay publishers did not each discover exactly one subscriber");
      }
      // 严格沿 MCAP 文件顺序发送，保留 payload bytes，不以 log timestamp 重排或重编码。
      for (const auto& frame : frames) {
        if (!publishers[frame.output_index]->Send(frame.isolated.payload.data(), frame.isolated.payload.size())) {
          throw std::runtime_error("replay raw eCAL send failed");
        }
        ++published[frame.output_index];
      }
    }
    eCAL::Finalize();
    initialized = false;
    WriteNewResult(plan.result, published);
    return 0;
  } catch (...) {
    if (initialized) eCAL::Finalize();
    throw;
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    return RunReplay(ParsePlan(argc, argv));
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
