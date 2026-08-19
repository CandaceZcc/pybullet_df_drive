// 阶段四 D：完成会话 Reader 必须拒绝被截断的 MCAP，不能让下游读取不完整记录。
#include "slope_sim/client/mcap_session_reader.hpp"
#include "slope_sim/client/mcap_session_writer.hpp"

#include <array>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../../common/sha256.hpp"
#include "slope_sim/client/v2_topics.hpp"
#include <mcap/mcap.hpp>
#include <unistd.h>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

template <std::size_t Size>
std::string Hex(const std::array<std::byte, Size>& bytes) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::byte value : bytes) {
    output << std::setw(2) << std::to_integer<unsigned int>(value);
  }
  return output.str();
}

slope_sim::client::v2::McapSessionIdentity TestIdentity() {
  return {
      {std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
       std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
       std::byte{0x88}, std::byte{0x99}, std::byte{0xaa}, std::byte{0xbb},
       std::byte{0xcc}, std::byte{0xdd}, std::byte{0xee}, std::byte{0xff}},
      {std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef}},
      7,
      "flat-obstacles-20",
      std::string(slope_sim::client::v2::kMid360PatternVersion),
      slope_sim::client::v2::kMid360PatternSha256,
  };
}

void WriteManifestFixture(
    const std::filesystem::path& path,
    const std::vector<std::byte>& descriptor,
    std::unordered_map<std::string, std::string> manifest) {
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("protobuf");
  options.noChunking = true;
  const auto opened = writer.open(path.string(), options);
  Require(opened.ok(), "manifest fixture cannot be opened");
  mcap::Schema schema("slope_sim.interfaces.v2", "protobuf", descriptor);
  writer.addSchema(schema);
  for (const auto& contract : slope_sim::client::v2::TopicContracts()) {
    mcap::Channel channel(
        contract.topic,
        "protobuf",
        schema.id,
        {{"type", contract.type_name}});
    writer.addChannel(channel);
  }
  const mcap::Metadata metadata{"slope_sim.session_manifest", std::move(manifest)};
  Require(writer.write(metadata).ok(), "manifest fixture metadata cannot be written");
  writer.close();
}

std::unordered_map<std::string, std::string> ValidManifest(
    const std::array<std::byte, 32>& descriptor_sha256) {
  return {
      {"simulation_session_id", "00112233445566778899aabbccddeeff"},
      {"descriptor_sha256", Hex(descriptor_sha256)},
      {"world_generation", "7"},
      {"scene_id", "flat-obstacles-20"},
      {"lidar_pattern_version", "livox-mid360-800000-v1"},
      {"lidar_pattern_sha256",
       "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"},
  };
}

std::vector<std::byte> ReadFixture(const std::string& relative_path) {
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  std::ifstream input(root / relative_path, std::ios::binary);
  Require(static_cast<bool>(input), "fixture cannot be opened");
  const std::string raw{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  return {reinterpret_cast<const std::byte*>(raw.data()),
          reinterpret_cast<const std::byte*>(raw.data()) + raw.size()};
}

}  // namespace

int main() {
  namespace fs = std::filesystem;
  using slope_sim::client::v2::McapSessionWriter;
  using slope_sim::client::v2::VerifyCompletedMcapSession;

  const fs::path directory = fs::temp_directory_path() / ("slope-sim-reader-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const fs::path recording = directory / "session.mcap";
  const auto descriptor = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  auto identity = TestIdentity();
  const std::string descriptor_text(reinterpret_cast<const char*>(descriptor.data()), descriptor.size());
  identity.descriptor_sha256 = stage4::Sha256(descriptor_text);
  const auto payload = ReadFixture("tests/fixtures/stage4/v2/WheelState.bin");
  {
    McapSessionWriter writer(recording, descriptor, identity);
    writer.Write("/sim/wheel/state", 3, 1000, 900, payload);
    writer.Finalize();
  }

  const auto verified = VerifyCompletedMcapSession(recording);
  Require(verified.simulation_session_id == identity.simulation_session_id,
          "Reader changed the simulation session identity");
  Require(verified.descriptor_sha256 == identity.descriptor_sha256,
          "Reader changed the descriptor identity");
  Require(verified.world_generation == identity.world_generation,
          "Reader changed the world generation");
  Require(verified.scene_id == identity.scene_id, "Reader changed the scene identity");
  Require(verified.lidar_pattern_version == "livox-mid360-800000-v1",
          "Reader changed the lidar pattern version");
  Require(Hex(verified.lidar_pattern_sha256) ==
              "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
          "Reader changed the lidar pattern digest");

  const auto session = slope_sim::client::v2::ReadCompletedMcapSession(recording);
  Require(session.frames.size() == 1, "Reader did not return the recorded raw frame");
  const auto& frame = session.frames.front();
  Require(frame.topic == "/sim/wheel/state" && frame.sequence == 3 &&
              frame.log_time_ns == 1000 && frame.publish_time_ns == 900,
          "Reader changed raw frame metadata");
  Require(frame.payload == payload,
          "Reader changed raw frame payload");

  const fs::path invalid = directory / "invalid.mcap";
  {
    McapSessionWriter writer(invalid, descriptor, identity);
    writer.Write("/sim/wheel/state", 3, 1000, 900, {std::byte{0x21}, std::byte{0x22}});
    writer.Finalize();
  }
  bool rejected_payload = false;
  try {
    static_cast<void>(slope_sim::client::v2::ReadCompletedMcapSession(invalid));
  } catch (const std::exception&) {
    rejected_payload = true;
  }
  Require(rejected_payload, "Reader accepted an invalid v2 payload");

  const fs::path truncated = directory / "truncated.mcap";
  fs::copy_file(recording, truncated);
  fs::resize_file(truncated, fs::file_size(truncated) - 1);
  bool rejected = false;
  try {
    static_cast<void>(VerifyCompletedMcapSession(truncated));
  } catch (const std::exception&) {
    rejected = true;
  }
  Require(rejected, "Reader accepted a truncated MCAP session");

  const auto valid_manifest = ValidManifest(identity.descriptor_sha256);
  for (const auto& [name, mutation] : std::vector<std::pair<std::string, std::string>>{
           {"missing", ""},
           {"malformed", "short"},
           {"different", std::string(64, '0')},
       }) {
    auto manifest = valid_manifest;
    if (name == "missing") {
      manifest.erase("lidar_pattern_sha256");
    } else {
      manifest["lidar_pattern_sha256"] = mutation;
    }
    const fs::path fixture = directory / ("pattern-" + name + ".mcap");
    WriteManifestFixture(fixture, descriptor, std::move(manifest));
    bool pattern_rejected = false;
    try {
      static_cast<void>(VerifyCompletedMcapSession(fixture));
    } catch (const std::exception&) {
      pattern_rejected = true;
    }
    Require(pattern_rejected,
            "Reader accepted a missing, malformed, or different lidar pattern digest");
  }

  for (const auto& [name, mutation] : std::vector<std::pair<std::string, std::string>>{
           {"missing", ""},
           {"malformed", "short"},
           {"different", "livox-mid360-800001-v1"},
       }) {
    auto manifest = valid_manifest;
    if (name == "missing") {
      manifest.erase("lidar_pattern_version");
    } else {
      manifest["lidar_pattern_version"] = mutation;
    }
    const fs::path fixture = directory / ("pattern-version-" + name + ".mcap");
    WriteManifestFixture(fixture, descriptor, std::move(manifest));
    bool version_rejected = false;
    try {
      static_cast<void>(VerifyCompletedMcapSession(fixture));
    } catch (const std::exception&) {
      version_rejected = true;
    }
    Require(version_rejected,
            "Reader accepted a missing, malformed, or different lidar pattern version");
  }

  fs::remove_all(directory);
  return 0;
}
