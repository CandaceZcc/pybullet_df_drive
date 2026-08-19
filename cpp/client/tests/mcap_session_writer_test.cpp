// 阶段四 C2：用官方 MCAP reader 锁定五 topic 原始 payload 与会话 manifest 的可读回性。
#include "slope_sim/client/mcap_session_writer.hpp"
#include "slope_sim/client/v2_topics.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <mcap/mcap.hpp>
#include <unistd.h>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

}  // namespace

int main() {
  namespace fs = std::filesystem;
  using slope_sim::client::v2::McapSessionIdentity;
  using slope_sim::client::v2::McapSessionWriter;
  using slope_sim::client::v2::TopicContracts;
  using slope_sim::client::v2::kMid360PatternSha256;
  using slope_sim::client::v2::kMid360PatternVersion;

  const fs::path directory = fs::temp_directory_path() / ("slope-sim-mcap-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const fs::path final_path = directory / "session.mcap";
  const std::vector<std::byte> descriptor{std::byte{0x0a}, std::byte{0x02}, std::byte{0x76}, std::byte{0x32}};
  const McapSessionIdentity identity{
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
      std::string(kMid360PatternVersion),
      kMid360PatternSha256,
  };
  std::unordered_map<std::string, std::vector<std::byte>> expected_payloads;
  std::unordered_map<std::string, std::string> expected_types;

  {
    McapSessionWriter writer(final_path, descriptor, identity);
    const auto& contracts = TopicContracts();
    for (std::size_t index = 0; index < contracts.size(); ++index) {
      const std::vector<std::byte> payload{
          std::byte{static_cast<unsigned char>(index + 1)},
          std::byte{static_cast<unsigned char>(index + 11)},
      };
      expected_payloads.emplace(contracts[index].topic, payload);
      expected_types.emplace(contracts[index].topic, contracts[index].type_name);
      writer.Write(contracts[index].topic, static_cast<std::uint32_t>(index + 1),
                   1000 + index, 900 + index, payload);
    }
    Require(!fs::exists(final_path), "final MCAP appeared before Finalize");
    writer.Finalize();
  }

  Require(fs::is_regular_file(final_path), "final MCAP is missing after Finalize");
  mcap::McapReader reader;
  const auto open_status = reader.open(final_path.string());
  Require(open_status.ok(), open_status.message.c_str());
  const auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  Require(summary_status.ok(), summary_status.message.c_str());
  Require(reader.footer().has_value(), "MCAP footer is missing");
  Require(reader.channels().size() == 5, "MCAP does not contain five channels");
  Require(reader.schemas().size() == 1, "MCAP does not contain one schema");
  const auto schema = reader.schemas().begin()->second;
  Require(schema->name == "slope_sim.interfaces.v2", "MCAP schema name differs");
  Require(schema->encoding == "protobuf", "MCAP schema encoding differs");
  Require(schema->data == descriptor, "MCAP schema bytes differ");
  const auto metadata = reader.metadataIndexes();
  const auto entries = metadata.equal_range("slope_sim.session_manifest");
  Require(entries.first != entries.second && std::next(entries.first) == entries.second,
          "MCAP session manifest is missing or duplicated");
  mcap::Record manifest_record{};
  const auto manifest_status = mcap::McapReader::ReadRecord(
      *reader.dataSource(), entries.first->second.offset, &manifest_record);
  mcap::Metadata manifest;
  Require(manifest_status.ok() &&
              mcap::McapReader::ParseMetadata(manifest_record, &manifest).ok(),
          "MCAP session manifest cannot be read");
  Require(manifest.metadata.at("lidar_pattern_version") ==
              "livox-mid360-800000-v1",
          "MCAP manifest lidar pattern version differs");
  Require(manifest.metadata.at("lidar_pattern_sha256") ==
              "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
          "MCAP manifest lidar pattern digest differs");

  std::size_t read_count = 0;
  for (const auto& view : reader.readMessages()) {
    Require(static_cast<bool>(view.channel), "MCAP message is missing a channel");
    Require(static_cast<bool>(view.schema), "MCAP message is missing a schema");
    Require(view.channel->messageEncoding == "protobuf", "MCAP encoding differs");
    Require(view.channel->metadata.at("type") == expected_types.at(view.channel->topic),
            "MCAP topic type differs");
    const auto expected = expected_payloads.at(view.channel->topic);
    Require(view.message.dataSize == expected.size(), "MCAP payload size differs");
    Require(std::equal(expected.begin(), expected.end(), view.message.data), "MCAP payload differs");
    ++read_count;
  }
  Require(read_count == expected_payloads.size(), "MCAP message count differs");
  fs::remove_all(directory);
  return 0;
}
