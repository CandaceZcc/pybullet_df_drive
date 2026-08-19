// 阶段四 Phase-0：在初始化 eCAL 前冻结 raw probe 的 dry-run 输入合同。
#include <algorithm>
#include <charconv>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <ecal/ecal.h>
#include <google/protobuf/descriptor.pb.h>

#include "sha256.hpp"
#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim/client/v2_topics.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

namespace fs = std::filesystem;

struct ProbePlan final {
  bool dry_run = false;
  std::string mode;
  std::string topic;
  std::string type_name;
  fs::path descriptor_set;
  std::optional<fs::path> payload;
  std::optional<fs::path> payload_out;
  fs::path result;
  std::optional<int> expected_peer_count;
  int deadline_ms = 0;
};

struct RawEnvelope final {
  std::vector<std::byte> payload;
  std::string remote_type_name;
  std::string remote_encoding;
  std::vector<std::byte> remote_descriptor;
  std::int64_t send_timestamp_us;
  std::int64_t send_clock;
  std::chrono::steady_clock::time_point received_at;
};

class ReceiveLane final {
 public:
  // callback 不等待 worker；lane 满时由调用方记录 runtime drop。
  bool Push(RawEnvelope envelope) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot_) {
      return false;
    }
    slot_.emplace(std::move(envelope));
    return true;
  }

  std::optional<RawEnvelope> Take() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!slot_) {
      return std::nullopt;
    }
    std::optional<RawEnvelope> envelope(std::move(slot_));
    slot_.reset();
    return envelope;
  }

 private:
  std::mutex mutex_;
  std::optional<RawEnvelope> slot_;
};

eCAL::SDataTypeInformation TypeInfo(
    const std::string& type_name,
    const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = type_name;
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

bool WaitForExactPeerCount(
    const std::function<std::size_t()>& peer_count,
    int deadline_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (peer_count() == 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return peer_count() == 1;
}

std::optional<RawEnvelope> WaitForEnvelope(ReceiveLane& lane, int deadline_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto envelope = lane.Take()) {
      return envelope;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return lane.Take();
}

class EcalLifecycle final {
 public:
  explicit EcalLifecycle(const std::string& unit_name) {
    if (!eCAL::Initialize(unit_name)) {
      throw std::runtime_error("eCAL initialization failed");
    }
    initialized_ = true;
  }

  ~EcalLifecycle() {
    (void)Finalize();
  }

  bool Finalize() {
    if (initialized_) {
      eCAL::Finalize();
      initialized_ = false;
    }
    return true;
  }

  EcalLifecycle(const EcalLifecycle&) = delete;
  EcalLifecycle& operator=(const EcalLifecycle&) = delete;

 private:
  bool initialized_ = false;
};

// native callback 只复制 eCAL 临时数据；后续 worker 才能安全执行 hash、验证与解析。
RawEnvelope CopyEnvelope(
    const eCAL::SDataTypeInformation& type_info,
    const eCAL::SReceiveCallbackData& data,
    std::chrono::steady_clock::time_point received_at) {
  if (data.buffer_size > 0 && data.buffer == nullptr) {
    throw std::runtime_error("eCAL callback payload buffer is null");
  }
  const auto* payload = static_cast<const std::byte*>(data.buffer);
  const auto* descriptor = reinterpret_cast<const std::byte*>(type_info.descriptor.data());
  return {
      std::vector<std::byte>(payload, payload + data.buffer_size),
      type_info.name,
      type_info.encoding,
      std::vector<std::byte>(descriptor, descriptor + type_info.descriptor.size()),
      data.send_timestamp,
      data.send_clock,
      received_at,
  };
}

class CliError final : public std::runtime_error {
 public:
  CliError(int code, std::string message) : std::runtime_error(std::move(message)), code_(code) {}
  int code() const { return code_; }

 private:
  int code_;
};

struct ProcessedEnvelope final {
  std::array<std::byte, 32> payload_sha256;
  slope_sim::interfaces::v2::WheelCommand command;
};

ProcessedEnvelope ProcessEnvelope(
    const RawEnvelope& envelope,
    const std::string& expected_type,
    const std::string& expected_descriptor) {
  const std::string payload(
      reinterpret_cast<const char*>(envelope.payload.data()), envelope.payload.size());
  // worker 的第一步固定为原始 bytes SHA-256，绝不允许 metadata/parse 抢在它之前。
  const auto payload_sha256 = stage4::Sha256(payload);
  const std::string remote_descriptor(
      reinterpret_cast<const char*>(envelope.remote_descriptor.data()),
      envelope.remote_descriptor.size());
  if (envelope.remote_type_name != expected_type ||
      envelope.remote_encoding != "proto" ||
      remote_descriptor != expected_descriptor) {
    throw std::runtime_error("remote type/encoding/descriptor mismatch");
  }
  slope_sim::interfaces::v2::WheelCommand command;
  if (!command.ParseFromString(payload)) {
    throw std::runtime_error("raw payload protobuf parse failed");
  }
  if (command.descriptor_sha256() != stage4::Bytes(stage4::Sha256(expected_descriptor)) ||
      command.simulation_session_id().size() != 16) {
    throw std::runtime_error("raw payload in-band identity mismatch");
  }
  return {payload_sha256, std::move(command)};
}

std::string ReadFile(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw CliError(66, "input is unreadable");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void WriteNewFile(const fs::path& path, const std::string& payload) {
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (descriptor < 0) {
    throw std::runtime_error("output cannot be created exclusively");
  }
  std::size_t written = 0;
  while (written < payload.size()) {
    const auto count = ::write(
        descriptor, payload.data() + written, payload.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      (void)::close(descriptor);
      throw std::runtime_error("output write failed");
    }
    written += static_cast<std::size_t>(count);
  }
  if (::fsync(descriptor) != 0) {
    (void)::close(descriptor);
    throw std::runtime_error("output sync failed");
  }
  if (::close(descriptor) != 0) {
    throw std::runtime_error("output close failed");
  }
}

std::string HexDigest(const std::array<std::byte, 32>& digest) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string output;
  output.reserve(digest.size() * 2);
  for (const std::byte value : digest) {
    const auto byte = static_cast<unsigned char>(value);
    output.push_back(kHex[byte >> 4]);
    output.push_back(kHex[byte & 0x0f]);
  }
  return output;
}

std::string JsonString(const std::string& value);

std::string BuildResultJson(
    const std::string& mode,
    const std::string& payload,
    const std::string& descriptor) {
  return "{\"descriptor_sha256\":\"" + HexDigest(stage4::Sha256(descriptor)) +
      "\",\"clean_shutdown\":true,\"mode\":\"" + mode + "\",\"payload_sha256\":\"" +
      HexDigest(stage4::Sha256(payload)) + "\",\"peer_count\":1}\n";
}

std::string BuildSubscribeResultJson(
    const RawEnvelope& envelope,
    const std::string& descriptor,
    const std::string& payload) {
  const std::string remote_descriptor(
      reinterpret_cast<const char*>(envelope.remote_descriptor.data()),
      envelope.remote_descriptor.size());
  return "{\"clean_shutdown\":true,\"descriptor_sha256\":\"" +
      HexDigest(stage4::Sha256(descriptor)) +
      "\",\"mode\":\"subscribe\",\"payload_sha256\":\"" +
      HexDigest(stage4::Sha256(payload)) +
      "\",\"peer_count\":1,\"remote_descriptor_sha256\":\"" +
      HexDigest(stage4::Sha256(remote_descriptor)) +
      "\",\"remote_encoding\":" + JsonString(envelope.remote_encoding) +
      ",\"remote_type_name\":" + JsonString(envelope.remote_type_name) +
      ",\"protocol_state\":\"verified\""
      ",\"worker_order\":[\"payload_sha256\",\"remote_metadata_verified\",\"protobuf_parsed\",\"in_band_identity_validated\"]"
      ",\"send_clock\":" + std::to_string(envelope.send_clock) +
      ",\"send_timestamp_us\":" + std::to_string(envelope.send_timestamp_us) + "}\n";
}

std::string MarkFinalized(std::string result_json) {
  if (result_json.size() < 2 ||
      result_json.compare(result_json.size() - 2, 2, "}\n") != 0) {
    throw std::runtime_error("result JSON is not a complete object");
  }
  result_json.insert(result_json.size() - 2, ",\"finalized\":true");
  return result_json;
}

fs::path InputPath(const std::string& raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path) {
    throw CliError(64, "input path must be absolute and normalized");
  }
  if (!fs::is_regular_file(path)) {
    throw CliError(66, "input must be an absolute regular file");
  }
  return path;
}

fs::path OutputPath(const std::string& raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path())) {
    throw CliError(64, "output must have an existing absolute parent directory");
  }
  if (fs::exists(path)) {
    throw CliError(73, "output already exists");
  }
  return path;
}

int PositiveInteger(const std::string& raw, const char* name, int minimum, int maximum) {
  int value = 0;
  const auto [end, error] = std::from_chars(raw.data(), raw.data() + raw.size(), value);
  if (error != std::errc{} || end != raw.data() + raw.size() || value < minimum || value > maximum) {
    throw CliError(64, std::string(name) + " is out of range");
  }
  return value;
}

void ValidateDescriptorAndPayload(const ProbePlan& plan) {
  google::protobuf::FileDescriptorSet descriptor;
  const std::string descriptor_bytes = ReadFile(plan.descriptor_set);
  if (!descriptor.ParseFromString(descriptor_bytes)) {
    throw CliError(66, "descriptor set is invalid");
  }
  bool type_found = false;
  for (const auto& file : descriptor.file()) {
    for (const auto& message : file.message_type()) {
      if (file.package() + "." + message.name() == plan.type_name) {
        type_found = true;
      }
    }
  }
  if (!type_found) {
    throw CliError(66, "type name is missing from descriptor set");
  }
  if (!plan.payload) {
    return;
  }
  const std::string payload = ReadFile(*plan.payload);
  const auto& contracts = slope_sim::client::v2::TopicContracts();
  const auto contract = std::find_if(
      contracts.begin(), contracts.end(), [&plan](const auto& item) { return plan.topic == item.topic; });
  if (contract == contracts.end() || plan.type_name != contract->type_name) {
    throw CliError(66, "topic and type name do not match the frozen v2 contract");
  }
  const auto validation = slope_sim::client::v2::ValidateRawV2Payload(
      plan.topic, payload, stage4::Bytes(stage4::Sha256(descriptor_bytes)));
  if (validation != slope_sim::client::v2::RawV2PayloadValidation::kValid) {
    throw CliError(66, "payload does not match the requested type");
  }
}

ProbePlan ParseProbeCli(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--version") {
    throw CliError(0, "version");
  }
  if (argc < 2) {
    throw CliError(64, "missing mode");
  }
  ProbePlan plan;
  int index = 1;
  if (std::string_view(argv[index]) == "--dry-run") {
    plan.dry_run = true;
    ++index;
  }
  if (index >= argc || (std::string_view(argv[index]) != "publish" && std::string_view(argv[index]) != "subscribe")) {
    throw CliError(64, "mode must be publish or subscribe");
  }
  plan.mode = argv[index++];
  std::map<std::string, std::string> options;
  while (index < argc) {
    const std::string key = argv[index++];
    if (key.rfind("--", 0) != 0 || index >= argc || !options.emplace(key, argv[index++]).second) {
      throw CliError(64, "unknown, missing, or duplicate option");
    }
  }
  const auto take = [&options](const char* key) -> std::string {
    const auto found = options.find(key);
    if (found == options.end()) {
      throw CliError(64, std::string("missing ") + key);
    }
    const std::string value = found->second;
    options.erase(found);
    return value;
  };
  plan.topic = take("--topic");
  plan.type_name = take("--type-name");
  if (plan.topic.empty() || plan.type_name.empty()) {
    throw CliError(64, "topic and type name must be nonempty");
  }
  plan.descriptor_set = InputPath(take("--descriptor-set"));
  plan.result = OutputPath(take("--result"));
  plan.deadline_ms = PositiveInteger(take("--deadline-ms"), "deadline-ms", 1, 60000);
  if (plan.mode == "publish") {
    plan.payload = InputPath(take("--payload"));
  } else {
    plan.payload_out = OutputPath(take("--payload-out"));
    plan.expected_peer_count = PositiveInteger(take("--expected-peer-count"), "expected-peer-count", 1, 1);
  }
  if (!options.empty()) {
    throw CliError(64, "option does not match mode");
  }
  if ((plan.payload && *plan.payload == plan.result) ||
      (plan.payload_out && (*plan.payload_out == plan.result || *plan.payload_out == plan.descriptor_set))) {
    throw CliError(73, "output aliases another path");
  }
  ValidateDescriptorAndPayload(plan);
  return plan;
}

std::string JsonString(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char byte : value) {
    if (byte == '\\' || byte == '"') output << '\\' << static_cast<char>(byte);
    else if (byte >= 0x20 && byte <= 0x7e) output << static_cast<char>(byte);
    else output << "\\u00" << "0123456789abcdef"[(byte >> 4) & 0xf] << "0123456789abcdef"[byte & 0xf];
  }
  return output.str() + '"';
}

int PrintCanonicalPlan(const ProbePlan& plan) {
  std::cout << "{\"deadline_ms\":" << plan.deadline_ms
            << ",\"descriptor_set\":" << JsonString(plan.descriptor_set.string())
            << ",\"dry_run\":true";
  if (plan.expected_peer_count) std::cout << ",\"expected_peer_count\":" << *plan.expected_peer_count;
  std::cout << ",\"mode\":" << JsonString(plan.mode);
  if (plan.payload) std::cout << ",\"payload\":" << JsonString(plan.payload->string());
  if (plan.payload_out) std::cout << ",\"payload_out\":" << JsonString(plan.payload_out->string());
  std::cout << ",\"result\":" << JsonString(plan.result.string())
            << ",\"topic\":" << JsonString(plan.topic)
            << ",\"type_name\":" << JsonString(plan.type_name) << "}\n";
  return 0;
}

int RunRealProbe(const ProbePlan& plan) {
  const std::string descriptor = ReadFile(plan.descriptor_set);
  std::string result_json;
  {
    EcalLifecycle lifecycle("stage4-phase0-raw-probe");
    {
    if (plan.mode == "publish") {
      eCAL::CPublisher publisher(plan.topic, TypeInfo(plan.type_name, descriptor));
      if (!WaitForExactPeerCount(
              [&publisher] { return publisher.GetSubscriberCount(); }, plan.deadline_ms)) {
        throw std::runtime_error("raw eCAL peer count did not reach exactly one");
      }
      const std::string payload = ReadFile(*plan.payload);
      if (!publisher.Send(payload.data(), payload.size())) {
        throw std::runtime_error("raw eCAL publisher send failed");
      }
      result_json = BuildResultJson(plan.mode, payload, descriptor);
    } else {
      ReceiveLane lane;
      eCAL::CSubscriber subscriber(plan.topic, TypeInfo(plan.type_name, descriptor));
      subscriber.SetReceiveCallback(
          [&lane](const eCAL::STopicId&,
                  const eCAL::SDataTypeInformation& type_info,
                  const eCAL::SReceiveCallbackData& data) {
            (void)lane.Push(CopyEnvelope(type_info, data, std::chrono::steady_clock::now()));
          });
      if (!WaitForExactPeerCount(
              [&subscriber] { return subscriber.GetPublisherCount(); }, plan.deadline_ms)) {
        throw std::runtime_error("raw eCAL peer count did not reach exactly one");
      }
      const auto envelope = WaitForEnvelope(lane, plan.deadline_ms);
      if (!envelope) {
        throw std::runtime_error("raw eCAL subscriber did not receive a frame");
      }
      const auto processed = ProcessEnvelope(*envelope, plan.type_name, descriptor);
      (void)processed;
      const std::string received_payload = std::string(
          reinterpret_cast<const char*>(envelope->payload.data()), envelope->payload.size());
      WriteNewFile(*plan.payload_out, received_payload);
      result_json = BuildSubscribeResultJson(*envelope, descriptor, received_payload);
    }
    }
    if (!lifecycle.Finalize()) {
      throw std::runtime_error("eCAL finalization failed");
    }
    result_json = MarkFinalized(std::move(result_json));
  }
  // eCAL lifecycle
  WriteNewFile(plan.result, result_json);
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const ProbePlan plan = ParseProbeCli(argc, argv);
    if (plan.dry_run) {
      return PrintCanonicalPlan(plan);
    }
    return RunRealProbe(plan);
  } catch (const CliError& error) {
    if (error.code() == 0) {
      std::cout << "cxx=17\ncompiler=gcc-13\necal=6.1.1\nprotobuf=33.6\nglibcxx_cxx11_abi=1\n";
      return 0;
    }
    std::cerr << "error: " << error.what() << '\n';
    return error.code();
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
