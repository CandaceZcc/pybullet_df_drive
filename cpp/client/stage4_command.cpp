// runSim v2：C++ Command 是 /sim/wheel/command 的唯一 publisher，并承载本机交互 socket。
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <openssl/crypto.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <ecal/ecal.h>

#include "../common/sha256.hpp"
#include "slope_sim/client/command_instance_lock.hpp"
#include "slope_sim/client/command_lease.hpp"
#include "slope_sim/client/command_socket_framer.hpp"
#include "slope_sim/client/command_twist.hpp"
#include "slope_sim/client/raw_wheel_command.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

namespace fs = std::filesystem;

std::string ReadInput(const std::string& raw_path) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("input must be an absolute normalized regular file");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("input cannot be opened");
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int PositiveDurationMs(const std::string& raw) {
  int value = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), value);
  // runSim 的交互 Command 必须覆盖数小时 GUI 会话；仍以有界 6 小时防止无期限
  // 后台 publisher，并保留 10 ms 节拍的确定性。
  if (error != std::errc{} || end != raw.data() + raw.size() || value < 100 ||
      value > 21600000 || value % 10 != 0) {
    throw std::invalid_argument("duration-ms must be a 10 ms multiple in [100, 21600000]");
  }
  return value;
}

int PositiveScheduleOffsetMs(const std::string& raw, const char* name) {
  int value = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), value);
  if (error != std::errc{} || end != raw.data() + raw.size() || value <= 0 ||
      value > 60000 || value % 10 != 0) {
    throw std::invalid_argument(std::string(name) + " must be a positive 10 ms multiple");
  }
  return value;
}

std::map<std::string, std::string> ParseOptions(int argc, char* argv[], int start) {
  std::map<std::string, std::string> options;
  for (int index = start; index < argc; index += 2) {
    if (index + 1 >= argc || std::string(argv[index]).rfind("--", 0) != 0 ||
        !options.emplace(argv[index], argv[index + 1]).second) {
      throw std::invalid_argument("options must be unique key/value pairs");
    }
  }
  return options;
}

struct ValidatedCommand final {
  std::string descriptor;
  std::string payload;
  std::string turn_payload;
  int duration_ms;
  int turn_at_ms;
  int stop_at_ms;
  bool scheduled;
  std::optional<std::pair<fs::path, fs::path>> coordination;
};

fs::path NewMarkerPath(const std::string& raw, const char* name) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path ||
      !fs::is_directory(path.parent_path()) || fs::exists(path)) {
    throw std::invalid_argument(std::string(name) + " must be a new absolute normalized marker");
  }
  return path;
}

void WriteMarker(const fs::path& path) {
  const int descriptor = open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  if (descriptor < 0) throw std::runtime_error("coordination marker already exists");
  (void)close(descriptor);
}

void ValidatePayload(
    const std::string& payload,
    std::string_view expected_descriptor_sha256,
    const char* name) {
  const auto outcome = slope_sim::client::v2::ValidateRawWheelCommand(
      payload, expected_descriptor_sha256);
  if (outcome == slope_sim::client::v2::RawWheelCommandValidation::kDescriptorDigestMismatch) {
    throw std::invalid_argument(std::string(name) + " descriptor digest differs from descriptor set");
  }
  if (outcome != slope_sim::client::v2::RawWheelCommandValidation::kValid) {
    throw std::invalid_argument(std::string(name) + " is not a valid WheelCommand");
  }
}

bool SameScheduleIdentity(
    const slope_sim::interfaces::v2::WheelCommand& first,
    const slope_sim::interfaces::v2::WheelCommand& second) {
  return first.simulation_session_id() == second.simulation_session_id() &&
      first.descriptor_sha256() == second.descriptor_sha256() &&
      first.world_generation() == second.world_generation() &&
      first.command_generation() == second.command_generation() &&
      first.source_id() == second.source_id() &&
      first.source_session_id() == second.source_session_id() &&
      first.robot_model() == second.robot_model();
}

ValidatedCommand ValidateInputs(const std::map<std::string, std::string>& options, bool require_result) {
  const int schedule_option_count =
      static_cast<int>(options.count("--turn-payload")) +
      static_cast<int>(options.count("--turn-at-ms")) +
      static_cast<int>(options.count("--stop-at-ms"));
  if (schedule_option_count != 0 && schedule_option_count != 3) {
    throw std::invalid_argument("schedule options must be provided together");
  }
  const bool scheduled = schedule_option_count == 3;
  const int coordination_option_count = static_cast<int>(options.count("--ready-file")) +
      static_cast<int>(options.count("--start-file")) +
      static_cast<int>(options.count("--expected-subscriber-count"));
  if (coordination_option_count != 0 && coordination_option_count != 3) {
    throw std::invalid_argument("coordinated start options must be provided together");
  }
  const bool coordinated = coordination_option_count == 3;
  const std::size_t expected_count = (require_result ? 5U : 3U) + (scheduled ? 3U : 0U) +
      (coordinated ? 3U : 0U);
  if (options.size() != expected_count || options.find("--descriptor-set") == options.end() ||
      options.find("--payload") == options.end() || options.find("--duration-ms") == options.end() ||
      (require_result && options.find("--result") == options.end())) {
    throw std::invalid_argument("command options are incomplete");
  }
  const std::string descriptor = ReadInput(options.at("--descriptor-set"));
  const std::string payload = ReadInput(options.at("--payload"));
  const int duration_ms = PositiveDurationMs(options.at("--duration-ms"));
  const std::string descriptor_sha256 = stage4::Bytes(stage4::Sha256(descriptor));
  ValidatePayload(payload, descriptor_sha256, "payload");
  std::optional<std::pair<fs::path, fs::path>> coordination;
  if (coordinated) {
    if (options.at("--expected-subscriber-count") != "2") {
      throw std::invalid_argument("expected-subscriber-count must be exactly 2");
    }
    const fs::path ready = NewMarkerPath(options.at("--ready-file"), "ready-file");
    const fs::path start = NewMarkerPath(options.at("--start-file"), "start-file");
    if (ready == start || ready.parent_path() != start.parent_path()) {
      throw std::invalid_argument("coordination markers must be distinct siblings");
    }
    coordination = std::make_pair(ready, start);
  }
  if (!scheduled) {
    return {descriptor, payload, "", duration_ms, 0, 0, false, coordination};
  }

  const std::string turn_payload = ReadInput(options.at("--turn-payload"));
  ValidatePayload(turn_payload, descriptor_sha256, "turn-payload");
  const int turn_at_ms = PositiveScheduleOffsetMs(options.at("--turn-at-ms"), "turn-at-ms");
  const int stop_at_ms = PositiveScheduleOffsetMs(options.at("--stop-at-ms"), "stop-at-ms");
  if (!(turn_at_ms < stop_at_ms && stop_at_ms < duration_ms)) {
    throw std::invalid_argument(
        "schedule must satisfy 0 < turn-at-ms < stop-at-ms < duration-ms");
  }

  slope_sim::interfaces::v2::WheelCommand first;
  slope_sim::interfaces::v2::WheelCommand turn;
  if (!first.ParseFromString(payload) || !turn.ParseFromString(turn_payload)) {
    throw std::runtime_error("validated command cannot be parsed");
  }
  if (!SameScheduleIdentity(first, turn)) {
    throw std::invalid_argument("schedule payload identity differs");
  }
  if (first.drive_wheel_speed_rad_s_size() != turn.drive_wheel_speed_rad_s_size() ||
      first.steering_wheel_speed_rad_s_size() != turn.steering_wheel_speed_rad_s_size()) {
    throw std::invalid_argument("schedule payload wheel shape differs");
  }
  return {descriptor, payload, turn_payload, duration_ms, turn_at_ms, stop_at_ms, true, coordination};
}

slope_sim::client::v2::WheelMotion MotionFor(
    const slope_sim::interfaces::v2::WheelCommand& command) {
  slope_sim::client::v2::WheelMotion motion;
  motion.drive_wheel_speed_rad_s.assign(
      command.drive_wheel_speed_rad_s().begin(), command.drive_wheel_speed_rad_s().end());
  motion.steering_wheel_speed_rad_s.assign(
      command.steering_wheel_speed_rad_s().begin(),
      command.steering_wheel_speed_rad_s().end());
  return motion;
}

eCAL::SDataTypeInformation CommandTypeInfo(const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = "slope_sim.interfaces.v2.WheelCommand";
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

/// Command 只拥有唯一 producer；Recorder 等被动消费者可以与 runtime 并存。
bool WaitForSubscriber(eCAL::CPublisher& publisher,
                       const std::chrono::steady_clock::time_point& deadline,
                       std::optional<int> exact_count) {
  while (std::chrono::steady_clock::now() < deadline) {
    const int count = publisher.GetSubscriberCount();
    if (exact_count.has_value() ? count == *exact_count : count >= 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const int count = publisher.GetSubscriberCount();
  return exact_count.has_value() ? count == *exact_count : count >= 1;
}

void WriteNewResult(
    const std::string& raw_path,
    int active_published_count,
    int safe_stop_published_count) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path())) {
    throw std::invalid_argument("result must have an existing absolute parent directory");
  }
  int descriptor = open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (descriptor < 0) {
    throw std::runtime_error("result must not already exist");
  }
  const int published_count = active_published_count + safe_stop_published_count;
  const std::string document = "{\"active_published_count\":" +
      std::to_string(active_published_count) + ",\"clean_shutdown\":true,\"published_count\":" +
      std::to_string(published_count) + ",\"safe_stop_published_count\":" +
      std::to_string(safe_stop_published_count) + ",\"transport\":\"ecal\"}\n";
  try {
    std::size_t written = 0;
    while (written < document.size()) {
      const ssize_t count = write(descriptor, document.data() + written, document.size() - written);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("command result write failed");
      written += static_cast<std::size_t>(count);
    }
    if (fsync(descriptor) != 0) {
      throw std::runtime_error("command result sync failed");
    }
    if (close(descriptor) != 0) {
      descriptor = -1;
      throw std::runtime_error("command result sync failed");
    }
    descriptor = -1;
  } catch (...) {
    if (descriptor >= 0) (void)close(descriptor);
    (void)unlink(path.c_str());
    throw;
  }
}

// 交互 socket 只需要 JSON object/string/number；手写小解析器避免引入数据面依赖。
struct JsonValue final {
  enum class Kind { kString, kNumber };
  Kind kind;
  std::string text;
};

class JsonObjectParser final {
 public:
  explicit JsonObjectParser(std::string_view input) : input_(input) {}

  std::map<std::string, JsonValue> Parse() {
    SkipWhitespace();
    Require('{');
    std::map<std::string, JsonValue> object;
    SkipWhitespace();
    if (Consume('}')) {
      RequireEnd();
      return object;
    }
    while (true) {
      SkipWhitespace();
      const std::string key = ParseString();
      SkipWhitespace();
      Require(':');
      SkipWhitespace();
      JsonValue value;
      if (Peek() == '"') {
        value = {JsonValue::Kind::kString, ParseString()};
      } else {
        value = {JsonValue::Kind::kNumber, ParseNumber()};
      }
      if (!object.emplace(key, std::move(value)).second) {
        throw std::invalid_argument("control JSON has duplicate field");
      }
      SkipWhitespace();
      if (Consume('}')) break;
      Require(',');
    }
    RequireEnd();
    return object;
  }

 private:
  char Peek() const {
    if (position_ == input_.size()) throw std::invalid_argument("control JSON is incomplete");
    return input_[position_];
  }

  bool Consume(char expected) {
    if (position_ < input_.size() && input_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  void Require(char expected) {
    if (!Consume(expected)) {
      if (position_ == input_.size()) throw std::invalid_argument("control JSON is incomplete");
      throw std::invalid_argument("control JSON syntax is invalid");
    }
  }

  void SkipWhitespace() {
    while (position_ < input_.size() &&
           (input_[position_] == ' ' || input_[position_] == '\n' ||
            input_[position_] == '\r' || input_[position_] == '\t')) {
      ++position_;
    }
  }

  void RequireEnd() {
    SkipWhitespace();
    if (position_ != input_.size()) throw std::invalid_argument("control JSON has trailing data");
  }

  std::string ParseString() {
    Require('"');
    std::string value;
    while (true) {
      if (position_ == input_.size()) throw std::invalid_argument("control JSON string is incomplete");
      const char current = input_[position_++];
      if (current == '"') return value;
      if (static_cast<unsigned char>(current) < 0x20U) {
        throw std::invalid_argument("control JSON string contains control character");
      }
      if (current != '\\') {
        value.push_back(current);
        continue;
      }
      if (position_ == input_.size()) throw std::invalid_argument("control JSON escape is incomplete");
      const char escaped = input_[position_++];
      switch (escaped) {
        case '"': value.push_back('"'); break;
        case '\\': value.push_back('\\'); break;
        case '/': value.push_back('/'); break;
        case 'b': value.push_back('\b'); break;
        case 'f': value.push_back('\f'); break;
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        case 'u': AppendUtf8(value, ParseUnicodeEscape()); break;
        default: throw std::invalid_argument("control JSON escape is invalid");
      }
    }
  }

  unsigned ParseHex4() {
    if (input_.size() - position_ < 4U) throw std::invalid_argument("control JSON is incomplete");
    unsigned value = 0;
    for (int index = 0; index < 4; ++index) {
      const char current = input_[position_++];
      value <<= 4U;
      if (current >= '0' && current <= '9') value |= static_cast<unsigned>(current - '0');
      else if (current >= 'a' && current <= 'f') value |= static_cast<unsigned>(current - 'a' + 10);
      else if (current >= 'A' && current <= 'F') value |= static_cast<unsigned>(current - 'A' + 10);
      else throw std::invalid_argument("control JSON unicode escape is invalid");
    }
    return value;
  }

  unsigned ParseUnicodeEscape() {
    const unsigned first = ParseHex4();
    if (first < 0xD800U || first > 0xDFFFU) return first;
    if (first >= 0xDC00U) throw std::invalid_argument("control JSON unicode surrogate is invalid");
    if (input_.size() - position_ < 2U) throw std::invalid_argument("control JSON is incomplete");
    if (input_[position_++] != '\\' || input_[position_++] != 'u') {
      throw std::invalid_argument("control JSON unicode surrogate is invalid");
    }
    const unsigned second = ParseHex4();
    if (second < 0xDC00U || second > 0xDFFFU) {
      throw std::invalid_argument("control JSON unicode surrogate is invalid");
    }
    return 0x10000U + ((first - 0xD800U) << 10U) + (second - 0xDC00U);
  }

  static void AppendUtf8(std::string& output, unsigned codepoint) {
    if (codepoint <= 0x7FU) output.push_back(static_cast<char>(codepoint));
    else if (codepoint <= 0x7FFU) {
      output.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    } else if (codepoint <= 0xFFFFU) {
      output.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
      output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    } else {
      output.push_back(static_cast<char>(0xF0U | (codepoint >> 18U)));
      output.push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU)));
      output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    }
  }

  std::string ParseNumber() {
    const std::size_t start = position_;
    (void)Consume('-');
    if (Consume('0')) {
      if (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        throw std::invalid_argument("control JSON number has leading zero");
      }
    } else {
      if (position_ == input_.size() || input_[position_] < '1' || input_[position_] > '9') {
        throw std::invalid_argument("control JSON value is not a number");
      }
      do { ++position_; } while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9');
    }
    if (Consume('.')) {
      const std::size_t fraction_start = position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') ++position_;
      if (position_ == fraction_start) throw std::invalid_argument("control JSON fraction is invalid");
    }
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) ++position_;
      const std::size_t exponent_start = position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') ++position_;
      if (position_ == exponent_start) throw std::invalid_argument("control JSON exponent is invalid");
    }
    return std::string(input_.substr(start, position_ - start));
  }

  std::string_view input_;
  std::size_t position_ = 0;
};

const JsonValue& RequireField(
    const std::map<std::string, JsonValue>& object,
    const char* name,
    JsonValue::Kind kind) {
  const auto found = object.find(name);
  if (found == object.end() || found->second.kind != kind) {
    throw std::invalid_argument("control JSON field is invalid");
  }
  return found->second;
}

void RequireExactFields(
    const std::map<std::string, JsonValue>& object,
    std::initializer_list<const char*> expected) {
  if (object.size() != expected.size()) throw std::invalid_argument("control JSON fields are invalid");
  for (const char* name : expected) {
    if (object.find(name) == object.end()) throw std::invalid_argument("control JSON fields are invalid");
  }
}

bool IsHex(const std::string& value, std::size_t length) {
  return value.size() == length && std::all_of(value.begin(), value.end(), [](unsigned char current) {
    return (current >= '0' && current <= '9') || (current >= 'a' && current <= 'f');
  });
}

long ParseNonnegativeLong(const JsonValue& value, const char* name, bool positive) {
  const std::string& raw = value.text;
  long parsed = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), parsed);
  if (error != std::errc{} || end != raw.data() + raw.size() || parsed < (positive ? 1 : 0)) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return parsed;
}

double ParseFiniteNumber(const JsonValue& value, const char* name) {
  char* end = nullptr;
  const double parsed = std::strtod(value.text.c_str(), &end);
  if (end != value.text.c_str() + value.text.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return parsed;
}

struct InteractiveAuthentication final {
  pid_t command_pid;
  uid_t command_uid;
  fs::path socket_path;
  std::string token;
};

InteractiveAuthentication ReadInteractiveAuthentication(const std::string& raw_record) {
  const fs::path record_path(raw_record);
  if (!record_path.is_absolute() || record_path.lexically_normal() != record_path) {
    throw std::invalid_argument("launch-record must be an absolute normalized regular file");
  }
  constexpr std::size_t kMaxLaunchRecordBytes = 4096U;
  int descriptor = open(record_path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) throw std::invalid_argument("launch-record cannot be opened securely");
  std::string record;
  try {
    struct stat before {};
    if (fstat(descriptor, &before) != 0 || !S_ISREG(before.st_mode) || before.st_uid != geteuid() ||
        (before.st_mode & 0777) != 0600 || before.st_size <= 0 ||
        static_cast<std::uintmax_t>(before.st_size) > kMaxLaunchRecordBytes) {
      throw std::invalid_argument("launch-record must be an euid-owned 0600 bounded regular file");
    }
    const std::size_t expected = static_cast<std::size_t>(before.st_size);
    record.resize(expected);
    std::size_t offset = 0;
    while (offset < expected) {
      const ssize_t count = read(descriptor, record.data() + offset, expected - offset);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::invalid_argument("launch-record changed while being read");
      offset += static_cast<std::size_t>(count);
    }
    char extra = '\0';
    ssize_t extra_count = 0;
    do {
      extra_count = read(descriptor, &extra, 1);
    } while (extra_count < 0 && errno == EINTR);
    struct stat after {};
    if (extra_count != 0 || fstat(descriptor, &after) != 0 || before.st_dev != after.st_dev ||
        before.st_ino != after.st_ino || before.st_size != after.st_size ||
        before.st_mode != after.st_mode || before.st_uid != after.st_uid) {
      throw std::invalid_argument("launch-record changed while being read");
    }
    if (close(descriptor) != 0) throw std::invalid_argument("launch-record close failed");
    descriptor = -1;
  } catch (...) {
    if (descriptor >= 0) (void)close(descriptor);
    throw;
  }
  const auto document = JsonObjectParser(record).Parse();
  RequireExactFields(document, {"command_pid", "command_uid", "orchestrator_pid", "protocol",
                                "session_id", "socket_path", "token"});
  const long command_pid = ParseNonnegativeLong(
      RequireField(document, "command_pid", JsonValue::Kind::kNumber), "command_pid", true);
  const long command_uid = ParseNonnegativeLong(
      RequireField(document, "command_uid", JsonValue::Kind::kNumber), "command_uid", false);
  if (command_uid != static_cast<long>(geteuid())) {
    throw std::invalid_argument("launch-record command uid does not match effective uid");
  }
  (void)ParseNonnegativeLong(
      RequireField(document, "orchestrator_pid", JsonValue::Kind::kNumber), "orchestrator_pid", true);
  if (RequireField(document, "protocol", JsonValue::Kind::kString).text !=
      "runsim-command-socket-v1") {
    throw std::invalid_argument("launch-record protocol is invalid");
  }
  const std::string& session_id = RequireField(document, "session_id", JsonValue::Kind::kString).text;
  const std::string& token = RequireField(document, "token", JsonValue::Kind::kString).text;
  if (!IsHex(session_id, 32U) || !IsHex(token, 64U)) {
    throw std::invalid_argument("launch-record authentication is invalid");
  }
  const fs::path socket_path(RequireField(document, "socket_path", JsonValue::Kind::kString).text);
  if (!socket_path.is_absolute() || socket_path.lexically_normal() != socket_path ||
      socket_path.filename() != "command.sock" || socket_path.parent_path() != record_path.parent_path() ||
      socket_path.native().size() > sizeof(sockaddr_un{}.sun_path) - 1U) {
    throw std::invalid_argument("launch-record socket_path is invalid");
  }
  return {static_cast<pid_t>(command_pid), static_cast<uid_t>(command_uid), socket_path, token};
}

bool IsAuthenticatedStopFrame(const std::string& payload, const std::string& expected_token);

class InteractiveSocketServer final {
 public:
  explicit InteractiveSocketServer(const InteractiveAuthentication& authentication)
      : path_(authentication.socket_path), expected_uid_(authentication.command_uid),
        expected_token_(authentication.token) {
    struct stat directory{};
    if (lstat(path_.parent_path().c_str(), &directory) != 0 || !S_ISDIR(directory.st_mode) ||
        (directory.st_mode & 0777) != 0700 || directory.st_uid != expected_uid_) {
      throw std::invalid_argument("interactive socket directory must be owned 0700 directory");
    }
    if (fs::exists(path_)) throw std::invalid_argument("interactive socket path already exists");
    server_fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (server_fd_ < 0) throw std::runtime_error("interactive socket creation failed");
    try {
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1U);
      if (bind(server_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
          chmod(path_.c_str(), 0600) != 0 || listen(server_fd_, 1) != 0 || !SetNonblocking(server_fd_)) {
        throw std::runtime_error("interactive socket bind failed");
      }
    } catch (...) {
      (void)close(server_fd_);
      server_fd_ = -1;
      (void)unlink(path_.c_str());
      throw;
    }
  }

  ~InteractiveSocketServer() {
    CloseClient();
    if (server_fd_ >= 0) (void)close(server_fd_);
    if (!path_.empty()) (void)unlink(path_.c_str());
  }

  InteractiveSocketServer(const InteractiveSocketServer&) = delete;
  InteractiveSocketServer& operator=(const InteractiveSocketServer&) = delete;

  enum class Event { kNone, kClosed, kInvalid, kMessage, kStop };

  Event Poll(std::vector<std::string>* messages) {
    messages->clear();
    if (client_fd_ < 0) AcceptAuthorizedClient();
    if (client_fd_ < 0) return Event::kNone;
    char buffer[1024];
    ssize_t received = 0;
    do {
      received = recv(client_fd_, buffer, sizeof(buffer), 0);
    } while (received < 0 && errno == EINTR);
    if (received == 0) {
      CloseClient();
      return Event::kClosed;
    }
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) return Event::kNone;
      CloseClient();
      return Event::kInvalid;
    }
    const auto framing = framer_.Append(
        std::string_view(buffer, static_cast<std::size_t>(received)), messages);
    if (framing == slope_sim::client::v2::CommandSocketFramer::Result::kOversize) {
      CloseClient();
      return Event::kInvalid;
    }
    for (std::size_t index = 0; index < messages->size(); ++index) {
      const std::string& frame = messages->at(index);
      try {
        (void)JsonObjectParser(frame).Parse();
      } catch (const std::invalid_argument&) {
        CloseClient();
        return Event::kInvalid;
      }
      // 已认证 stop 是终止边界，后续同批字节无需再作 JSON 解析。
      if (IsAuthenticatedStopFrame(frame, expected_token_)) {
        messages->resize(index + 1U);
        return Event::kStop;
      }
    }
    return framing == slope_sim::client::v2::CommandSocketFramer::Result::kFrame
        ? Event::kMessage : Event::kNone;
  }

 private:
  static bool SetNonblocking(int descriptor) {
    const int flags = fcntl(descriptor, F_GETFL);
    return flags >= 0 && fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) == 0;
  }

  void AcceptAuthorizedClient() {
    while (true) {
      const int descriptor = accept4(server_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
      if (descriptor < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        throw std::runtime_error("interactive socket accept failed");
      }
      ucred credential{};
      socklen_t size = sizeof(credential);
      if (getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credential, &size) == 0 &&
          size == sizeof(credential) && credential.uid == expected_uid_) {
        client_fd_ = descriptor;
        return;
      }
      (void)close(descriptor);
    }
  }

  void CloseClient() {
    if (client_fd_ >= 0) {
      (void)close(client_fd_);
      client_fd_ = -1;
    }
    framer_.Clear();
  }

  fs::path path_;
  uid_t expected_uid_;
  std::string expected_token_;
  int server_fd_ = -1;
  int client_fd_ = -1;
  slope_sim::client::v2::CommandSocketFramer framer_;
};

slope_sim::client::v2::RobotCommandShape ShapeFor(
    const slope_sim::interfaces::v2::WheelCommand& command) {
  using slope_sim::client::v2::RobotCommandShape;
  if (command.robot_model() == "df_front" || command.robot_model() == "df_mid" ||
      command.robot_model() == "df_back") {
    if (command.drive_wheel_speed_rad_s_size() == 2 && command.steering_wheel_speed_rad_s_size() == 0) {
      return RobotCommandShape::kDifferential;
    }
  } else if (command.robot_model() == "active_steering_4wd" &&
             command.drive_wheel_speed_rad_s_size() == 4 &&
             command.steering_wheel_speed_rad_s_size() == 2) {
    return RobotCommandShape::kActiveSteering4wd;
  }
  throw std::invalid_argument("interactive payload has unsupported robot wheel shape");
}

void ValidateInteractiveIdentity(const slope_sim::interfaces::v2::WheelCommand& command) {
  if (command.simulation_session_id().size() != 16 || command.source_session_id().size() != 16 ||
      command.descriptor_sha256().size() != 32 || command.world_generation() == 0 ||
      command.command_generation() == 0 || command.source_id().empty()) {
    throw std::invalid_argument("interactive payload identity is incomplete");
  }
  (void)ShapeFor(command);
}

bool ConstantTimeTokenMatches(const std::string& actual, const std::string& expected) {
  return actual.size() == expected.size() &&
      CRYPTO_memcmp(actual.data(), expected.data(), expected.size()) == 0;
}

enum class InteractiveMessage { kTarget, kStatus, kStop };

InteractiveMessage ValidateInteractiveMessage(
    const std::string& payload,
    const std::string& expected_token,
    float* linear_velocity_m_s,
    float* angular_velocity_rad_s) {
  if (payload.size() > 1024U) throw std::invalid_argument("control payload exceeds maximum size");
  const auto document = JsonObjectParser(payload).Parse();
  for (const auto& [key, ignored] : document) {
    (void)ignored;
    std::string lowered = key;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char current) {
      return static_cast<char>(std::tolower(current));
    });
    if (lowered.find("pointcloud") != std::string::npos) {
      throw std::invalid_argument("pointcloud fields are forbidden on control socket");
    }
  }
  const std::string& kind = RequireField(document, "kind", JsonValue::Kind::kString).text;
  const std::string& token = RequireField(document, "token", JsonValue::Kind::kString).text;
  if (!ConstantTimeTokenMatches(token, expected_token)) {
    throw std::invalid_argument("control message token does not match session");
  }
  if (kind == "target") {
    RequireExactFields(document, {"kind", "token", "linear_velocity_m_s", "angular_velocity_rad_s"});
    *linear_velocity_m_s = static_cast<float>(ParseFiniteNumber(
        RequireField(document, "linear_velocity_m_s", JsonValue::Kind::kNumber), "linear_velocity_m_s"));
    *angular_velocity_rad_s = static_cast<float>(ParseFiniteNumber(
        RequireField(document, "angular_velocity_rad_s", JsonValue::Kind::kNumber), "angular_velocity_rad_s"));
    return InteractiveMessage::kTarget;
  }
  if (kind == "status") {
    RequireExactFields(document, {"kind", "token", "state"});
    const std::string& state = RequireField(document, "state", JsonValue::Kind::kString).text;
    if (state != "launching" && state != "ready" && state != "active" && state != "safe_stop" &&
        state != "stopping" && state != "closed") {
      throw std::invalid_argument("status state is invalid");
    }
    return InteractiveMessage::kStatus;
  }
  if (kind == "stop") {
    RequireExactFields(document, {"kind", "token", "reason"});
    const std::string& reason = RequireField(document, "reason", JsonValue::Kind::kString).text;
    if (reason.empty() || reason.size() > 128U) throw std::invalid_argument("stop reason is invalid");
    return InteractiveMessage::kStop;
  }
  throw std::invalid_argument("control message kind is invalid");
}

bool IsAuthenticatedStopFrame(const std::string& payload, const std::string& expected_token) {
  float ignored_linear_velocity_m_s = 0.0F;
  float ignored_angular_velocity_rad_s = 0.0F;
  try {
    return ValidateInteractiveMessage(
        payload, expected_token, &ignored_linear_velocity_m_s, &ignored_angular_velocity_rad_s) ==
        InteractiveMessage::kStop;
  } catch (const std::invalid_argument&) {
    return false;
  }
}

int RunRealCommand(const std::map<std::string, std::string>& options) {
  const ValidatedCommand command = ValidateInputs(options, true);
  const int deadline_ms = [&options] {
    const auto found = options.find("--deadline-ms");
    if (found == options.end()) {
      throw std::invalid_argument("command requires deadline-ms");
    }
    return PositiveDurationMs(found->second);
  }();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  slope_sim::client::v2::CommandInstanceLock lock;
  if (!eCAL::Initialize("slope-sim-stage4-command")) {
    throw std::runtime_error("eCAL initialization failed");
  }
  bool initialized = true;
  try {
    slope_sim::interfaces::v2::WheelCommand input_command;
    if (!input_command.ParseFromString(command.payload)) {
      throw std::runtime_error("validated command cannot be parsed");
    }
    slope_sim::interfaces::v2::WheelCommand turn_command;
    if (command.scheduled && !turn_command.ParseFromString(command.turn_payload)) {
      throw std::runtime_error("validated turn command cannot be parsed");
    }
    slope_sim::client::v2::WheelCommandLease lease(
        static_cast<std::size_t>(input_command.drive_wheel_speed_rad_s_size()),
        static_cast<std::size_t>(input_command.steering_wheel_speed_rad_s_size()));
    const auto straight_motion = MotionFor(input_command);
    const auto turn_motion = command.scheduled ? MotionFor(turn_command) : straight_motion;
    const slope_sim::client::v2::WheelMotion stop_motion{
        std::vector<float>(straight_motion.drive_wheel_speed_rad_s.size(), 0.0F),
        std::vector<float>(straight_motion.steering_wheel_speed_rad_s.size(), 0.0F)};
    if (!command.scheduled) {
      lease.Renew(straight_motion, std::chrono::milliseconds(0));
    }
    int active_published_count = 0;
    int safe_stop_published_count = 0;
    {
      eCAL::CPublisher publisher("/sim/wheel/command", CommandTypeInfo(command.descriptor));
      const std::optional<int> expected_count = command.coordination.has_value()
          ? std::optional<int>(2) : std::nullopt;
      if (!WaitForSubscriber(publisher, deadline, expected_count)) {
        throw std::runtime_error("command peer count did not reach required consumers");
      }
      if (command.coordination.has_value()) {
        const auto& [ready, start_marker] = *command.coordination;
        WriteMarker(ready);
        while (!fs::exists(start_marker)) {
          if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("shared start marker did not arrive before deadline");
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!WaitForSubscriber(publisher, deadline, expected_count)) {
          throw std::runtime_error("command peer count changed before shared start");
        }
      }
      const auto start = std::chrono::steady_clock::now();
      for (int offset_ms = 0; offset_ms < command.duration_ms; offset_ms += 10) {
        std::this_thread::sleep_until(start + std::chrono::milliseconds(offset_ms));
        if (command.scheduled) {
          const auto& scheduled_motion = offset_ms < command.turn_at_ms
              ? straight_motion
              : (offset_ms < command.stop_at_ms ? turn_motion : stop_motion);
          lease.Renew(scheduled_motion, std::chrono::milliseconds(offset_ms));
        }
        const auto values = lease.Decision(std::chrono::milliseconds(offset_ms));
        slope_sim::interfaces::v2::WheelCommand frame(input_command);
        frame.clear_drive_wheel_speed_rad_s();
        frame.clear_steering_wheel_speed_rad_s();
        for (const float value : values.drive_wheel_speed_rad_s) {
          frame.add_drive_wheel_speed_rad_s(value);
        }
        for (const float value : values.steering_wheel_speed_rad_s) {
          frame.add_steering_wheel_speed_rad_s(value);
        }
        frame.set_sequence(input_command.sequence() + static_cast<std::uint64_t>(offset_ms / 10));
        const std::string payload = frame.SerializeAsString();
        if (!publisher.Send(payload.data(), payload.size())) {
          throw std::runtime_error("command raw eCAL send failed");
        }
        if (lease.state() == slope_sim::client::v2::CommandLeaseState::kTimedOut) {
          ++safe_stop_published_count;
        } else {
          ++active_published_count;
        }
      }
    }
    eCAL::Finalize();
    initialized = false;
    WriteNewResult(options.at("--result"), active_published_count, safe_stop_published_count);
    return 0;
  } catch (...) {
    if (initialized) {
      eCAL::Finalize();
    }
    throw;
  }
}

int RunInteractiveCommand(const std::map<std::string, std::string>& options) {
  if (options.size() != 6U || options.find("--launch-record") == options.end()) {
    throw std::invalid_argument("interactive command options are incomplete");
  }
  auto validated_options = options;
  const std::string launch_record = validated_options.extract("--launch-record").mapped();
  const ValidatedCommand command = ValidateInputs(validated_options, true);
  const auto deadline_option = options.find("--deadline-ms");
  if (deadline_option == options.end()) throw std::invalid_argument("command requires deadline-ms");
  const int deadline_ms = PositiveDurationMs(deadline_option->second);
  const InteractiveAuthentication authentication = ReadInteractiveAuthentication(launch_record);
  // bind 前自验 PID；启动记录不能被另一个同 uid 进程拿来监听。
  if (getpid() != authentication.command_pid) {
    throw std::invalid_argument("interactive command PID does not match launch record");
  }
  slope_sim::interfaces::v2::WheelCommand template_command;
  if (!template_command.ParseFromString(command.payload)) {
    throw std::runtime_error("validated command cannot be parsed");
  }
  ValidateInteractiveIdentity(template_command);
  const auto shape = ShapeFor(template_command);
  const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  slope_sim::client::v2::CommandInstanceLock lock;
  InteractiveSocketServer server(authentication);
  if (!eCAL::Initialize("slope-sim-stage4-command")) {
    throw std::runtime_error("eCAL initialization failed");
  }
  bool initialized = true;
  try {
    slope_sim::client::v2::WheelCommandLease lease(
        static_cast<std::size_t>(template_command.drive_wheel_speed_rad_s_size()),
        static_cast<std::size_t>(template_command.steering_wheel_speed_rad_s_size()));
    slope_sim::client::v2::TwistCommandConverter converter(shape);
    const slope_sim::client::v2::WheelMotion zero_motion{
        std::vector<float>(static_cast<std::size_t>(template_command.drive_wheel_speed_rad_s_size()), 0.0F),
        std::vector<float>(static_cast<std::size_t>(template_command.steering_wheel_speed_rad_s_size()), 0.0F)};
    bool has_target = false;
    bool stop_requested = false;
    int active_published_count = 0;
    int safe_stop_published_count = 0;
    eCAL::CPublisher publisher("/sim/wheel/command", CommandTypeInfo(command.descriptor));
    const auto start = std::chrono::steady_clock::now();
    for (int offset_ms = 0; offset_ms < command.duration_ms; offset_ms += 10) {
      std::this_thread::sleep_until(start + std::chrono::milliseconds(offset_ms));
      const auto now = std::chrono::steady_clock::now();
      if (now >= start_deadline) {
        throw std::runtime_error("interactive command deadline elapsed");
      }
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
      std::vector<std::string> socket_messages;
      const auto event = server.Poll(&socket_messages);
      if (event == InteractiveSocketServer::Event::kClosed || event == InteractiveSocketServer::Event::kInvalid) {
        has_target = false;
      } else if (event == InteractiveSocketServer::Event::kMessage ||
                 event == InteractiveSocketServer::Event::kStop) {
        (void)slope_sim::client::v2::ProcessCommandSocketFramesUntilTerminal(
            socket_messages, [&](const std::string& socket_message) {
              try {
                float linear_velocity_m_s = 0.0F;
                float angular_velocity_rad_s = 0.0F;
                const InteractiveMessage message = ValidateInteractiveMessage(
                    socket_message, authentication.token, &linear_velocity_m_s, &angular_velocity_rad_s);
                if (message == InteractiveMessage::kTarget) {
                  lease.Renew(
                      converter.Convert(linear_velocity_m_s, angular_velocity_rad_s),
                      elapsed);
                  has_target = true;
                } else if (message == InteractiveMessage::kStop) {
                  has_target = false;
                  stop_requested = true;
                  return true;
                }
              } catch (const std::invalid_argument&) {
                // 未认证、超限或未知消息均 fail closed，但不让攻击者终止 publisher。
                has_target = false;
              }
              return false;
            });
      }
      const auto values = has_target ? lease.Decision(elapsed) : zero_motion;
      if (has_target && lease.state() == slope_sim::client::v2::CommandLeaseState::kTimedOut) {
        has_target = false;
      }
      slope_sim::interfaces::v2::WheelCommand frame(template_command);
      frame.clear_drive_wheel_speed_rad_s();
      frame.clear_steering_wheel_speed_rad_s();
      for (const float value : values.drive_wheel_speed_rad_s) frame.add_drive_wheel_speed_rad_s(value);
      for (const float value : values.steering_wheel_speed_rad_s) frame.add_steering_wheel_speed_rad_s(value);
      frame.set_timestamp_ns(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count()));
      frame.set_sequence(template_command.sequence() + static_cast<std::uint64_t>(offset_ms / 10));
      const std::string payload = frame.SerializeAsString();
      if (!publisher.Send(payload.data(), payload.size())) {
        throw std::runtime_error("command raw eCAL send failed");
      }
      if (has_target) {
        ++active_published_count;
      } else {
        ++safe_stop_published_count;
      }
      if (stop_requested) break;
    }
    eCAL::Finalize();
    initialized = false;
    WriteNewResult(options.at("--result"), active_published_count, safe_stop_published_count);
    return 0;
  } catch (...) {
    if (initialized) eCAL::Finalize();
    throw;
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc < 2) {
      throw std::invalid_argument("command mode is missing");
    }
    if (std::string(argv[1]) == "--dry-run") {
      (void)ValidateInputs(ParseOptions(argc, argv, 2), false);
      std::cout << "validated command: /sim/wheel/command 100Hz\n";
      return 0;
    }
    if (std::string(argv[1]) == "--interactive") {
      return RunInteractiveCommand(ParseOptions(argc, argv, 2));
    }
    return RunRealCommand(ParseOptions(argc, argv, 1));
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 66;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
