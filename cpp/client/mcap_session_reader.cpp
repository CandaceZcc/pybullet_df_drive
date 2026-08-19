// 阶段四 D：使用官方 MCAP reader 拒绝不完整、错 schema 或身份不完整的会话。
#include "slope_sim/client/mcap_session_reader.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <mcap/mcap.hpp>

#include "../common/sha256.hpp"
#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace slope_sim::client::v2 {
namespace {

constexpr char kManifestName[] = "slope_sim.session_manifest";

std::vector<std::byte> ParseHex(std::string_view value, std::size_t byte_count, std::string_view name) {
  if (value.size() != byte_count * 2) {
    throw std::runtime_error(std::string(name) + " has an invalid length");
  }
  std::vector<std::byte> bytes(byte_count);
  for (std::size_t index = 0; index < byte_count; ++index) {
    unsigned int parsed = 0;
    const auto [end, error] = std::from_chars(
        value.data() + index * 2, value.data() + index * 2 + 2, parsed, 16);
    if (error != std::errc{} || end != value.data() + index * 2 + 2) {
      throw std::runtime_error(std::string(name) + " is not hexadecimal");
    }
    bytes[index] = static_cast<std::byte>(parsed);
  }
  return bytes;
}

std::string MetadataValue(const mcap::Metadata& metadata, std::string_view name) {
  const auto value = metadata.metadata.find(std::string(name));
  if (value == metadata.metadata.end() || value->second.empty()) {
    throw std::runtime_error("MCAP session manifest is incomplete");
  }
  return value->second;
}

}  // namespace

McapSessionIdentity VerifyCompletedMcapSession(const std::filesystem::path& path) {
  namespace fs = std::filesystem;
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("MCAP session must be an absolute normalized regular file");
  }

  mcap::McapReader reader;
  const auto open_status = reader.open(path.string());
  if (!open_status.ok()) throw std::runtime_error("MCAP session cannot be opened: " + open_status.message);
  const auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!summary_status.ok() || !reader.footer().has_value()) {
    throw std::runtime_error("MCAP session is not finalized");
  }

  const auto schemas = reader.schemas();
  if (schemas.size() != 1 || schemas.begin()->second->name != "slope_sim.interfaces.v2" ||
      schemas.begin()->second->encoding != "protobuf") {
    throw std::runtime_error("MCAP session schema is not the frozen v2 schema");
  }
  const auto schema_id = schemas.begin()->first;
  const auto channels = reader.channels();
  const auto& contracts = TopicContracts();
  if (channels.size() != contracts.size()) throw std::runtime_error("MCAP session has an unexpected channel count");
  std::unordered_map<std::string, const TopicContract*> expected;
  for (const auto& contract : contracts) expected.emplace(contract.topic, &contract);
  for (const auto& [id, channel] : channels) {
    (void)id;
    const auto contract = expected.find(channel->topic);
    if (contract == expected.end() || channel->schemaId != schema_id ||
        channel->messageEncoding != "protobuf") {
      throw std::runtime_error("MCAP session has an unexpected channel");
    }
    const auto type = channel->metadata.find("type");
    if (type == channel->metadata.end() || type->second != contract->second->type_name) {
      throw std::runtime_error("MCAP session channel type differs from the frozen v2 contract");
    }
    expected.erase(contract);
  }
  if (!expected.empty()) throw std::runtime_error("MCAP session is missing a frozen v2 channel");

  const auto entries = reader.metadataIndexes().equal_range(kManifestName);
  if (entries.first == entries.second || std::next(entries.first) != entries.second) {
    throw std::runtime_error("MCAP session must contain exactly one manifest");
  }
  mcap::Record record{};
  const auto record_status = mcap::McapReader::ReadRecord(
      *reader.dataSource(), entries.first->second.offset, &record);
  mcap::Metadata metadata;
  if (!record_status.ok() || !mcap::McapReader::ParseMetadata(record, &metadata).ok()) {
    throw std::runtime_error("MCAP session manifest cannot be read");
  }

  const auto session = ParseHex(MetadataValue(metadata, "simulation_session_id"), 16,
                                "simulation_session_id");
  const auto descriptor = ParseHex(MetadataValue(metadata, "descriptor_sha256"), 32,
                                   "descriptor_sha256");
  const std::string lidar_pattern_version = MetadataValue(metadata, "lidar_pattern_version");
  if (lidar_pattern_version != kMid360PatternVersion) {
    throw std::runtime_error("lidar_pattern_version differs from the frozen MID-360 pattern");
  }
  const auto lidar_pattern = ParseHex(MetadataValue(metadata, "lidar_pattern_sha256"), 32,
                                      "lidar_pattern_sha256");
  if (!std::equal(lidar_pattern.begin(), lidar_pattern.end(), kMid360PatternSha256.begin())) {
    throw std::runtime_error("lidar_pattern_sha256 differs from the frozen MID-360 pattern");
  }
  std::uint64_t world_generation = 0;
  const std::string world_text = MetadataValue(metadata, "world_generation");
  const auto [world_end, world_error] = std::from_chars(
      world_text.data(), world_text.data() + world_text.size(), world_generation);
  if (world_error != std::errc{} || world_end != world_text.data() + world_text.size() ||
      world_generation == 0) {
    throw std::runtime_error("world_generation is invalid");
  }

  McapSessionIdentity identity{};
  std::copy(session.begin(), session.end(), identity.simulation_session_id.begin());
  std::copy(descriptor.begin(), descriptor.end(), identity.descriptor_sha256.begin());
  identity.world_generation = world_generation;
  identity.scene_id = MetadataValue(metadata, "scene_id");
  identity.lidar_pattern_version = lidar_pattern_version;
  std::copy(lidar_pattern.begin(), lidar_pattern.end(), identity.lidar_pattern_sha256.begin());
  return identity;
}

CompletedMcapSession ReadCompletedMcapSession(const std::filesystem::path& path) {
  CompletedMcapSession session{VerifyCompletedMcapSession(path), {}};
  mcap::McapReader reader;
  const auto open_status = reader.open(path.string());
  if (!open_status.ok()) throw std::runtime_error("verified MCAP session cannot be reopened");
  const auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!summary_status.ok() || reader.schemas().size() != 1) {
    throw std::runtime_error("verified MCAP session summary cannot be reopened");
  }
  const auto& schema = reader.schemas().begin()->second;
  const std::string descriptor(reinterpret_cast<const char*>(schema->data.data()), schema->data.size());
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  if (!std::equal(digest.begin(), digest.end(),
                  reinterpret_cast<const char*>(session.identity.descriptor_sha256.data()))) {
    throw std::runtime_error("MCAP schema descriptor differs from session manifest");
  }

  for (const auto& view : reader.readMessages([](const mcap::Status& status) {
         throw std::runtime_error("MCAP message cannot be read: " + status.message);
       })) {
    if (!view.channel || view.message.data == nullptr) {
      throw std::runtime_error("MCAP session contains an invalid raw frame");
    }
    const std::string_view payload(reinterpret_cast<const char*>(view.message.data), view.message.dataSize);
    if (ValidateRawV2Payload(view.channel->topic, payload, digest) != RawV2PayloadValidation::kValid) {
      throw std::runtime_error("MCAP session contains an invalid v2 payload");
    }
    session.frames.push_back({
        view.channel->topic,
        std::vector<std::byte>(view.message.data, view.message.data + view.message.dataSize),
        view.message.sequence,
        view.message.logTime,
        view.message.publishTime,
    });
  }
  return session;
}

}  // namespace slope_sim::client::v2
