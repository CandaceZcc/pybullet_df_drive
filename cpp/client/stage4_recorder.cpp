// 阶段四 C2：独立 eCAL Recorder，验证五个冻结 topic 后将原始 bytes 原子写为 MCAP。
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <ecal/ecal.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "../common/sha256.hpp"
#include "slope_sim/client/atomic_segment.hpp"
#include "slope_sim/client/interactive_recorder_window.hpp"
#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim/client/recorder_session.hpp"
#include "slope_sim/client/v2_topics.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

namespace fs = std::filesystem;
using slope_sim::client::v2::McapSessionIdentity;
using slope_sim::client::v2::InteractiveRecorderWindow;
using slope_sim::client::v2::RecordedRawFrame;
using slope_sim::client::v2::RecorderEnqueueResult;
using slope_sim::client::v2::RecorderSession;
using slope_sim::client::v2::RecorderSessionState;
using slope_sim::client::v2::TopicContract;

struct RecorderPlan final {
  fs::path descriptor_set;
  fs::path output;
  fs::path result;
  McapSessionIdentity identity;
  std::array<int, 5> expected_counts{};
  int deadline_ms = 0;
  bool interactive = false;
  fs::path control_socket;
  std::array<std::byte, 32> control_token{};
};

struct Receipt final {
  const TopicContract* contract = nullptr;
  std::atomic<int> accepted_count{0};
  std::atomic<int> rejected_count{0};
  std::optional<std::uint64_t> next_sequence;
};

struct ParsedFrame final {
  std::array<std::byte, 16> simulation_session_id;
  std::uint64_t world_generation = 0;
  std::uint32_t sequence = 0;
  std::uint64_t timestamp_ns = 0;
};

std::string ReadInput(const std::string& raw_path) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("input must be an absolute normalized regular file");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("input cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

fs::path NewOutputPath(const std::string& raw_path, const char* name) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path()) ||
      fs::exists(path)) {
    throw std::invalid_argument(std::string(name) + " must be a new file below an existing absolute directory");
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

int RecorderDeadlineMilliseconds(const std::string& raw) {
  constexpr int kMaximumOfflineDeadlineMs = 6 * 60 * 60 * 1000;
  int value = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), value);
  if (error != std::errc{} || end != raw.data() + raw.size() || value < 1 ||
      value > kMaximumOfflineDeadlineMs) {
    throw std::invalid_argument("deadline-ms must be an integer in [1, 21600000]");
  }
  return value;
}

template <std::size_t Size>
std::array<std::byte, Size> ParseHex(const std::string& raw, const char* name) {
  if (raw.size() != Size * 2) {
    throw std::invalid_argument(std::string(name) + " must contain " +
                                std::to_string(Size * 2) + " lowercase hex characters");
  }
  std::array<std::byte, Size> bytes{};
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    unsigned int value = 0;
    const auto [end, error] = std::from_chars(
        raw.data() + index * 2, raw.data() + index * 2 + 2, value, 16);
    if (error != std::errc{} || end != raw.data() + index * 2 + 2) {
      throw std::invalid_argument(std::string(name) + " must contain " +
                                  std::to_string(Size * 2) + " lowercase hex characters");
    }
    bytes[index] = static_cast<std::byte>(value);
  }
  return bytes;
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

fs::path NewControlSocketPath(const std::string& raw_path) {
  const fs::path path(raw_path);
  struct stat parent{};
  if (!path.is_absolute() || path.lexically_normal() != path || path.filename().empty() ||
      ::lstat(path.parent_path().c_str(), &parent) != 0 || !S_ISDIR(parent.st_mode) ||
      S_ISLNK(parent.st_mode) || parent.st_uid != ::getuid() || (parent.st_mode & 0777) != 0700 ||
      fs::exists(path) || path.string().size() >= sizeof(sockaddr_un::sun_path)) {
    throw std::invalid_argument(
        "control-socket must be a new short path below a 0700 directory owned by this user");
  }
  return path;
}

RecorderPlan ParsePlan(int argc, char* argv[], bool interactive = false) {
  const auto options = ParseOptions(argc, argv);
  const bool has_pattern_version = options.find("--lidar-pattern-version") != options.end();
  const bool has_pattern_sha256 = options.find("--lidar-pattern-sha256") != options.end();
  if (!has_pattern_version || !has_pattern_sha256) {
    throw std::invalid_argument(
        "lidar pattern identity requires --lidar-pattern-version and --lidar-pattern-sha256");
  }
  static constexpr std::array<const char*, 9> kRequired{
      "--descriptor-set", "--scene-id", "--simulation-session-id", "--world-generation",
      "--lidar-pattern-version", "--lidar-pattern-sha256", "--output", "--deadline-ms", "--result"};
  for (const char* option : kRequired) {
    if (options.find(option) == options.end()) throw std::invalid_argument("recorder options are incomplete");
  }
  static constexpr std::array<const char*, 5> kExplicitCountOptions{
      "--expected-wheel-command-count", "--expected-wheel-state-count",
      "--expected-lidar-points-count", "--expected-rtk-state-count",
      "--expected-imu-attitude-count"};
  const auto uniform_count = options.find("--expected-count");
  const auto duration = options.find("--duration-ms");
  const std::size_t explicit_count_options = std::count_if(
      kExplicitCountOptions.begin(), kExplicitCountOptions.end(),
      [&options](const char* option) { return options.find(option) != options.end(); });
  if (explicit_count_options != 0 && explicit_count_options != kExplicitCountOptions.size()) {
    throw std::invalid_argument("explicit recorder window requires all five topic counts");
  }
  const bool has_uniform_count = uniform_count != options.end();
  const bool has_duration = duration != options.end();
  const bool has_explicit_counts = explicit_count_options == kExplicitCountOptions.size();
  if (!interactive && static_cast<int>(has_uniform_count) + static_cast<int>(has_duration) +
          static_cast<int>(has_explicit_counts) != 1) {
    throw std::invalid_argument("recorder requires exactly one window option");
  }
  if (interactive && (has_uniform_count || has_duration || has_explicit_counts ||
      options.find("--control-socket") == options.end() || options.find("--control-token") == options.end())) {
    throw std::invalid_argument("interactive recorder requires only control-socket and control-token window options");
  }
  const std::size_t expected_option_count =
      kRequired.size() + (interactive ? 2 : (has_explicit_counts ? kExplicitCountOptions.size() : 1));
  if (options.size() != expected_option_count) {
    throw std::invalid_argument("recorder options are incomplete");
  }
  RecorderPlan plan;
  plan.descriptor_set = fs::path(options.at("--descriptor-set"));
  const std::string descriptor = ReadInput(plan.descriptor_set.string());
  plan.identity.simulation_session_id = ParseHex<16>(
      options.at("--simulation-session-id"), "simulation-session-id");
  const auto digest = stage4::Sha256(descriptor);
  for (std::size_t index = 0; index < digest.size(); ++index) {
    plan.identity.descriptor_sha256[index] = digest[index];
  }
  plan.identity.world_generation = static_cast<std::uint64_t>(PositiveInteger(
      options.at("--world-generation"), "world-generation"));
  plan.identity.scene_id = options.at("--scene-id");
  if (plan.identity.scene_id.empty()) throw std::invalid_argument("scene-id must be nonempty");
  plan.identity.lidar_pattern_version = options.at("--lidar-pattern-version");
  if (plan.identity.lidar_pattern_version != slope_sim::client::v2::kMid360PatternVersion) {
    throw std::invalid_argument("lidar-pattern-version differs from the frozen MID-360 pattern");
  }
  plan.identity.lidar_pattern_sha256 = ParseHex<32>(
      options.at("--lidar-pattern-sha256"), "lidar-pattern-sha256");
  if (plan.identity.lidar_pattern_sha256 != slope_sim::client::v2::kMid360PatternSha256) {
    throw std::invalid_argument("lidar-pattern-sha256 differs from the frozen MID-360 pattern");
  }
  plan.output = NewOutputPath(options.at("--output"), "output");
  plan.result = NewOutputPath(options.at("--result"), "result");
  if (plan.output == plan.result) throw std::invalid_argument("output and result must differ");
  if (interactive) {
    plan.interactive = true;
    plan.control_socket = NewControlSocketPath(options.at("--control-socket"));
    plan.control_token = ParseHex<32>(options.at("--control-token"), "control-token");
  } else if (has_uniform_count) {
    plan.expected_counts.fill(PositiveInteger(uniform_count->second, "expected-count"));
  } else if (has_duration) {
    const int duration_ms = PositiveInteger(duration->second, "duration-ms");
    if (duration_ms % 100 != 0) {
      throw std::invalid_argument("duration-ms must align with the frozen 10Hz sensor cadence");
    }
    // 五话题固定为 Command/WheelState 100 Hz 与三类传感器 10 Hz，无额外配置层。
    plan.expected_counts = {
        duration_ms / 10, duration_ms / 10, duration_ms / 100, duration_ms / 100, duration_ms / 100};
  } else {
    for (std::size_t index = 0; index < kExplicitCountOptions.size(); ++index) {
      plan.expected_counts[index] = PositiveInteger(
          options.at(kExplicitCountOptions[index]), kExplicitCountOptions[index] + 2);
    }
  }
  plan.deadline_ms = RecorderDeadlineMilliseconds(options.at("--deadline-ms"));
  return plan;
}

eCAL::SDataTypeInformation TopicTypeInfo(const TopicContract& contract, const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = contract.type_name;
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

const google::protobuf::FieldDescriptor& RequireField(
    const google::protobuf::Descriptor& descriptor, const char* name) {
  const auto* field = descriptor.FindFieldByName(name);
  if (field == nullptr) throw std::runtime_error("frozen v2 protobuf field is missing");
  return *field;
}

/// 统一 validator 已确认 payload 合法后，仅提取 MCAP 索引和会话绑定字段。
ParsedFrame ParseValidatedFrame(const TopicContract& contract, std::string_view payload) {
  const auto* descriptor = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(contract.type_name);
  const auto* prototype = descriptor == nullptr
      ? nullptr
      : google::protobuf::MessageFactory::generated_factory()->GetPrototype(descriptor);
  if (prototype == nullptr) throw std::runtime_error("frozen v2 protobuf type is unavailable");
  std::unique_ptr<google::protobuf::Message> message(prototype->New());
  if (!message->ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
    throw std::runtime_error("validated raw payload cannot be parsed");
  }
  const auto* reflection = message->GetReflection();
  const auto& session_field = RequireField(*descriptor, "simulation_session_id");
  const std::string session = reflection->GetString(*message, &session_field);
  if (session.size() != 16) throw std::runtime_error("validated raw payload session length changed");
  ParsedFrame frame;
  for (std::size_t index = 0; index < frame.simulation_session_id.size(); ++index) {
    frame.simulation_session_id[index] = static_cast<std::byte>(static_cast<unsigned char>(session[index]));
  }
  frame.world_generation = reflection->GetUInt64(*message, &RequireField(*descriptor, "world_generation"));
  const std::uint64_t sequence = reflection->GetUInt64(*message, &RequireField(*descriptor, "sequence"));
  if (sequence > UINT32_MAX) throw std::runtime_error("v2 sequence exceeds MCAP sequence range");
  frame.sequence = static_cast<std::uint32_t>(sequence);
  const auto* timestamp = descriptor->FindFieldByName("timestamp_ns");
  if (timestamp == nullptr) timestamp = descriptor->FindFieldByName("timebase_ns");
  if (timestamp == nullptr) throw std::runtime_error("frozen v2 timestamp field is missing");
  frame.timestamp_ns = reflection->GetUInt64(*message, timestamp);
  return frame;
}

bool MatchesIdentity(const ParsedFrame& frame, const McapSessionIdentity& identity) {
  return frame.simulation_session_id == identity.simulation_session_id &&
      frame.world_generation == identity.world_generation;
}

/// Recorder 专用 Unix socket：只接受同 uid 编排器的一次 start/stop/status 控制消息。
class InteractiveRecorderSocket final {
 public:
  enum class Event { kNone, kStart, kStop, kInvalid };

  InteractiveRecorderSocket(fs::path path, std::array<std::byte, 32> token)
      : path_(std::move(path)), token_(token) {
    descriptor_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (descriptor_ < 0) throw std::runtime_error("interactive recorder socket create failed");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const std::string native_path = path_.string();
    std::copy(native_path.begin(), native_path.end(), address.sun_path);
    if (::bind(descriptor_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
        ::listen(descriptor_, 4) != 0) {
      throw std::runtime_error("interactive recorder socket bind/listen failed");
    }
  }

  ~InteractiveRecorderSocket() {
    if (descriptor_ >= 0) (void)::close(descriptor_);
    (void)::unlink(path_.c_str());
  }

  InteractiveRecorderSocket(const InteractiveRecorderSocket&) = delete;
  InteractiveRecorderSocket& operator=(const InteractiveRecorderSocket&) = delete;

  Event Poll(std::string_view state) {
    pollfd ready{descriptor_, POLLIN, 0};
    const int poll_result = ::poll(&ready, 1, 0);
    if (poll_result < 0) return errno == EINTR ? Event::kNone : Event::kInvalid;
    if (poll_result == 0) return Event::kNone;
    const int client = ::accept4(descriptor_, nullptr, nullptr, SOCK_CLOEXEC);
    if (client < 0) return errno == EINTR ? Event::kNone : Event::kInvalid;
    const auto close_client = [&client] { (void)::close(client); };
    struct ucred peer{};
    socklen_t peer_size = sizeof(peer);
    if (::getsockopt(client, SOL_SOCKET, SO_PEERCRED, &peer, &peer_size) != 0 || peer.uid != ::getuid()) {
      close_client();
      return Event::kInvalid;
    }
    std::array<char, 1025> message{};
    const ssize_t received = ::recv(client, message.data(), message.size(), 0);
    if (received <= 0 || received >= 1024) {
      close_client();
      return Event::kInvalid;
    }
    const std::string_view payload(message.data(), static_cast<std::size_t>(received));
    const auto kind = JsonStringField(payload, "kind");
    const auto token = JsonStringField(payload, "token");
    if (!kind.has_value() || !token.has_value() || !TokenMatches(*token)) {
      close_client();
      return Event::kInvalid;
    }
    if (*kind == "status") {
      const std::string response = "{\"state\":\"" + std::string(state) + "\"}\n";
      (void)::send(client, response.data(), response.size(), MSG_NOSIGNAL);
      close_client();
      return Event::kNone;
    }
    close_client();
    if (*kind == "start") return Event::kStart;
    if (*kind == "stop") return Event::kStop;
    return Event::kInvalid;
  }

 private:
  static std::optional<std::string_view> JsonStringField(std::string_view document, std::string_view name) {
    const std::string needle = "\"" + std::string(name) + "\"";
    const std::size_t name_offset = document.find(needle);
    if (name_offset == std::string_view::npos) return std::nullopt;
    std::size_t offset = name_offset + needle.size();
    while (offset < document.size() && (document[offset] == ' ' || document[offset] == '\n' ||
                                        document[offset] == '\r' || document[offset] == '\t')) ++offset;
    if (offset >= document.size() || document[offset++] != ':') return std::nullopt;
    while (offset < document.size() && (document[offset] == ' ' || document[offset] == '\n' ||
                                        document[offset] == '\r' || document[offset] == '\t')) ++offset;
    if (offset >= document.size() || document[offset++] != '\"') return std::nullopt;
    const std::size_t end = document.find('\"', offset);
    if (end == std::string_view::npos || document.substr(offset, end - offset).find('\\') != std::string_view::npos) {
      return std::nullopt;
    }
    return document.substr(offset, end - offset);
  }

  bool TokenMatches(std::string_view supplied) const {
    static constexpr char kHex[] = "0123456789abcdef";
    if (supplied.size() != token_.size() * 2) return false;
    unsigned char difference = 0;
    for (std::size_t index = 0; index < token_.size(); ++index) {
      const unsigned char value = std::to_integer<unsigned char>(token_[index]);
      difference |= static_cast<unsigned char>(supplied[index * 2] ^ kHex[value >> 4]);
      difference |= static_cast<unsigned char>(supplied[index * 2 + 1] ^ kHex[value & 0x0f]);
    }
    return difference == 0;
  }

  fs::path path_;
  std::array<std::byte, 32> token_;
  int descriptor_ = -1;
};

std::string_view InteractiveStateName(InteractiveRecorderWindow::State state) {
  switch (state) {
    case InteractiveRecorderWindow::State::kAwaitingStart: return "awaiting_start";
    case InteractiveRecorderWindow::State::kAwaitingStartBoundary: return "awaiting_start_boundary";
    case InteractiveRecorderWindow::State::kRecording: return "recording";
    case InteractiveRecorderWindow::State::kAwaitingFinalBoundary: return "awaiting_final_boundary";
    case InteractiveRecorderWindow::State::kReadyToFinalize: return "ready_to_finalize";
    case InteractiveRecorderWindow::State::kSafeStopRequired: return "safe_stop_required";
  }
  return "safe_stop_required";
}

/// 成功和故障都以独占结果文件结束，供上层编排区分正常完成与必须停车。
std::string JsonString(std::string_view value) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string escaped{"\""};
  for (const unsigned char character : value) {
    switch (character) {
      case '\"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\b': escaped += "\\b"; break;
      case '\f': escaped += "\\f"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:
        if (character < 0x20) {
          escaped += "\\u00";
          escaped += kHex[character >> 4];
          escaped += kHex[character & 0x0f];
        } else {
          escaped += static_cast<char>(character);
        }
    }
  }
  escaped += '\"';
  return escaped;
}

void WriteNewResult(
    const fs::path& path,
    const fs::path& mcap_path,
    const std::array<Receipt, 5>& receipts,
    bool clean_shutdown,
    std::string_view fault_reason = {},
    bool interactive = false) {
  std::string topics;
  int recorded_count = 0;
  for (std::size_t index = 0; index < receipts.size(); ++index) {
    if (index != 0) topics += ',';
    const int count = receipts[index].accepted_count.load();
    recorded_count += count;
    topics += "\"" + std::string(receipts[index].contract->topic) + "\":" + std::to_string(count);
  }
  const std::string document = "{\"clean_shutdown\":" +
      std::string(clean_shutdown ? "true" : "false") +
      (clean_shutdown
           ? ""
           : ",\"fault_reason\":" + JsonString(fault_reason) +
                 ",\"safe_stop_required\":true") +
      ",\"mcap\":" + JsonString(mcap_path.string()) +
      ",\"recorded_count\":" + std::to_string(recorded_count) +
      (interactive ? ",\"exportable\":" + std::string(clean_shutdown ? "true" : "false") : "") +
      ",\"role\":\"recorder\",\"topics\":{" + topics + "}}\n";
  slope_sim::client::v2::AtomicSegment manifest(path);
  manifest.Append(std::vector<std::byte>(reinterpret_cast<const std::byte*>(document.data()),
                                         reinterpret_cast<const std::byte*>(document.data()) + document.size()));
  manifest.Finalize();
}

int RunRecorder(const RecorderPlan& plan) {
  const std::string descriptor = ReadInput(plan.descriptor_set.string());
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  const auto& contracts = slope_sim::client::v2::TopicContracts();
  std::array<Receipt, 5> receipts{};
  for (std::size_t index = 0; index < receipts.size(); ++index) receipts[index].contract = &contracts[index];
  RecorderSession session(4096, plan.output, std::vector<std::byte>(
      reinterpret_cast<const std::byte*>(descriptor.data()),
      reinterpret_cast<const std::byte*>(descriptor.data()) + descriptor.size()), plan.identity);
  std::unique_ptr<InteractiveRecorderWindow> interactive_window;
  std::unique_ptr<InteractiveRecorderSocket> control_socket;
  if (plan.interactive) {
    interactive_window = std::make_unique<InteractiveRecorderWindow>();
    control_socket = std::make_unique<InteractiveRecorderSocket>(plan.control_socket, plan.control_token);
  }
  std::mutex receipt_mutex;
  std::mutex fault_mutex;
  std::atomic<bool> faulted{false};
  std::string fault_reason;
  std::atomic<std::uint64_t> last_command_monotonic_ns{0};
  const auto latch_fault = [&fault_mutex, &faulted, &fault_reason](std::string reason) {
    {
      std::lock_guard<std::mutex> lock(fault_mutex);
      if (fault_reason.empty()) fault_reason = std::move(reason);
    }
    faulted = true;
  };
  const auto current_fault_reason = [&fault_mutex, &fault_reason]() {
    std::lock_guard<std::mutex> lock(fault_mutex);
    return fault_reason;
  };
  if (!eCAL::Initialize("slope-sim-stage4-recorder")) throw std::runtime_error("eCAL initialization failed");
  bool initialized = true;
  try {
    {
      std::array<std::unique_ptr<eCAL::CSubscriber>, 5> subscribers;
      for (std::size_t index = 0; index < subscribers.size(); ++index) {
        Receipt* const receipt = &receipts[index];
        subscribers[index] = std::make_unique<eCAL::CSubscriber>(
            receipt->contract->topic, TopicTypeInfo(*receipt->contract, descriptor));
        subscribers[index]->SetReceiveCallback(
            [&digest, &plan, &session, &receipts, &receipt_mutex, &latch_fault, &interactive_window,
             &last_command_monotonic_ns, receipt, index](
                const eCAL::STopicId&, const eCAL::SDataTypeInformation& type_info,
                const eCAL::SReceiveCallbackData& data) {
              // eCAL callback 的 buffer 只在回调期间有效，先复制再执行任何验证或排队。
              if (data.buffer_size > 0 && data.buffer == nullptr) {
                ++receipt->rejected_count;
                latch_fault(std::string(receipt->contract->topic) + " callback buffer is null");
                return;
              }
              const auto* bytes = static_cast<const std::byte*>(data.buffer);
              std::vector<std::byte> payload(bytes, bytes + data.buffer_size);
              const std::string_view raw(reinterpret_cast<const char*>(payload.data()), payload.size());
              if (type_info.name != receipt->contract->type_name || type_info.encoding != "proto" ||
                  stage4::Bytes(stage4::Sha256(type_info.descriptor)) != digest) {
                ++receipt->rejected_count;
                latch_fault(std::string(receipt->contract->topic) + " eCAL metadata is invalid");
                return;
              }
              if (slope_sim::client::v2::ValidateRawV2Payload(receipt->contract->topic, raw, digest) !=
                  slope_sim::client::v2::RawV2PayloadValidation::kValid) {
                ++receipt->rejected_count;
                latch_fault(std::string(receipt->contract->topic) + " payload is invalid");
                return;
              }
              try {
                const ParsedFrame parsed = ParseValidatedFrame(*receipt->contract, raw);
                if (!MatchesIdentity(parsed, plan.identity)) {
                  ++receipt->rejected_count;
                  latch_fault(std::string(receipt->contract->topic) + " identity does not match recorder plan");
                  return;
                }
                std::lock_guard<std::mutex> lock(receipt_mutex);
                // 录制窗口不能用重复帧填充：每个冻结 topic 只接受连续 sequence。
                if (receipt->next_sequence.has_value() && parsed.sequence != *receipt->next_sequence) {
                  ++receipt->rejected_count;
                  latch_fault(std::string(receipt->contract->topic) + " sequence is not continuous");
                  return;
                }
                receipt->next_sequence = static_cast<std::uint64_t>(parsed.sequence) + 1;
                if (index == 0) {
                  last_command_monotonic_ns.store(static_cast<std::uint64_t>(
                      std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch()).count()));
                }
                std::vector<RecordedRawFrame> committed;
                if (interactive_window) {
                  committed = interactive_window->Observe(index, {
                      receipt->contract->topic, std::move(payload), parsed.sequence,
                      parsed.timestamp_ns, parsed.timestamp_ns});
                  if (interactive_window->state() == InteractiveRecorderWindow::State::kSafeStopRequired) {
                    latch_fault("interactive Recorder boundary is invalid");
                    return;
                  }
                } else {
                  committed.push_back({receipt->contract->topic, std::move(payload), parsed.sequence,
                                       parsed.timestamp_ns, parsed.timestamp_ns});
                }
                for (auto& frame : committed) {
                  const auto target = std::find_if(receipts.begin(), receipts.end(), [&frame](const Receipt& item) {
                    return item.contract->topic == frame.topic;
                  });
                  if (target == receipts.end()) {
                    latch_fault("interactive Recorder committed an unknown topic");
                    return;
                  }
                  const RecorderEnqueueResult result = session.Enqueue(std::move(frame));
                  if (result != RecorderEnqueueResult::kAccepted) {
                    latch_fault(std::string(target->contract->topic) +
                        (result == RecorderEnqueueResult::kOverflow
                             ? " Recorder queue overflowed"
                             : " Recorder session is not recording"));
                    return;
                  }
                  ++target->accepted_count;
                }
              } catch (const std::exception& error) {
                ++receipt->rejected_count;
                latch_fault(
                    std::string(receipt->contract->topic) + " callback failed: " + error.what());
              }
            });
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(plan.deadline_ms);
      bool complete = false;
      while (std::chrono::steady_clock::now() < deadline && !faulted.load()) {
        if (control_socket) {
          const auto event = control_socket->Poll(InteractiveStateName(interactive_window->state()));
          if (event == InteractiveRecorderSocket::Event::kStart && !interactive_window->RequestStart()) {
            latch_fault("interactive Recorder start request is invalid");
          } else if (event == InteractiveRecorderSocket::Event::kStop && !interactive_window->RequestStop()) {
            latch_fault("interactive Recorder stop request is invalid");
          } else if (event == InteractiveRecorderSocket::Event::kInvalid) {
            latch_fault("interactive Recorder control message is invalid");
          }
        }
        while (session.DrainOne()) {
        }
        if (session.state() != RecorderSessionState::kRecording) {
          latch_fault("Recorder session cannot continue writing");
        }
        if (interactive_window) {
          const auto state = interactive_window->state();
          const auto now_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
          const std::uint64_t command_ns = last_command_monotonic_ns.load();
          if ((state == InteractiveRecorderWindow::State::kRecording ||
               state == InteractiveRecorderWindow::State::kAwaitingFinalBoundary) &&
              (command_ns == 0 || now_ns > command_ns + 250000000ULL)) {
            interactive_window->RequireSafeStop();
            latch_fault("interactive Recorder lost /sim/wheel/command");
          }
          complete = state == InteractiveRecorderWindow::State::kReadyToFinalize;
          if (state == InteractiveRecorderWindow::State::kSafeStopRequired) {
            latch_fault("interactive Recorder requires safe stop");
          }
          for (const auto& receipt : receipts) {
            if (receipt.rejected_count.load() != 0) {
              latch_fault(std::string(receipt.contract->topic) + " rejected a frame");
            }
          }
        } else {
          complete = true;
          for (std::size_t index = 0; index < receipts.size(); ++index) {
            const auto& receipt = receipts[index];
            const int accepted = receipt.accepted_count.load();
            complete = complete && accepted == plan.expected_counts[index] && receipt.rejected_count.load() == 0;
            if (accepted > plan.expected_counts[index]) {
              latch_fault(std::string(receipt.contract->topic) + " exceeded its expected count");
            }
          }
        }
        if (complete || faulted.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (!complete || faulted.load()) {
        if (!faulted.load()) latch_fault(plan.interactive
            ? "interactive Recorder deadline expired before final 10Hz boundary"
            : "Recorder deadline expired before exact topic counts");
        throw std::runtime_error(current_fault_reason());
      }
      if (!session.Finalize() || session.state() != RecorderSessionState::kFinalized) {
        latch_fault("Recorder could not finalize MCAP session");
        throw std::runtime_error(current_fault_reason());
      }
    }
    eCAL::Finalize();
    initialized = false;
    WriteNewResult(plan.result, plan.output, receipts, true, {}, plan.interactive);
    return 0;
  } catch (...) {
    if (initialized) eCAL::Finalize();
    // Recorder 已无法保证无损会话；以独占 result 将安全停车请求交给进程编排层。
    try {
      std::string reason = current_fault_reason();
      if (reason.empty()) reason = "Recorder failed without a detailed reason";
      WriteNewResult(plan.result, plan.output, receipts, false, reason, plan.interactive);
    } catch (const std::exception&) {
      // 原始失败优先；result 写入失败同样以非零退出暴露给 Supervisor。
    }
    throw;
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc > 1 && std::string_view(argv[1]) == "--interactive") {
      return RunRecorder(ParsePlan(argc - 1, argv + 1, true));
    }
    return RunRecorder(ParsePlan(argc, argv));
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
