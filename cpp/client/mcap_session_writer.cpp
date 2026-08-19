// 阶段四 C2：官方 MCAP writer 与同目录 AtomicSegment 的最小会话持久化实现。
#include "slope_sim/client/mcap_session_writer.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

// 官方 MCAP header-only 库要求恰好一个翻译单元提供实现符号。
#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include "slope_sim/client/atomic_segment.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace slope_sim::client::v2 {
namespace {

std::string ToHex(const std::byte* bytes, std::size_t size) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    output << std::setw(2) << std::to_integer<unsigned int>(bytes[index]);
  }
  return output.str();
}

void RequireSuccess(const mcap::Status& status, std::string_view action) {
  if (!status.ok()) {
    throw std::runtime_error(std::string(action) + ": " + status.message);
  }
}

/// 将 MCAP 库的流写入回调桥接到已具备 fsync/rename 合同的 AtomicSegment。
class AtomicSegmentOutput final : public mcap::IWritable {
 public:
  explicit AtomicSegmentOutput(AtomicSegment& segment) : segment_(segment) {}

  void end() override {}

  std::uint64_t size() const override {
    return size_;
  }

 protected:
  void handleWrite(const std::byte* data, std::uint64_t size) override {
    if (size == 0) {
      return;
    }
    std::vector<std::byte> bytes(data, data + size);
    segment_.Append(bytes);
    size_ += size;
  }

 private:
  AtomicSegment& segment_;
  std::uint64_t size_ = 0;
};

}  // namespace

class McapSessionWriter::Impl final {
 public:
  Impl(std::filesystem::path final_path,
       std::vector<std::byte> descriptor_set,
       McapSessionIdentity identity)
      : segment(std::move(final_path)), output(segment) {
    if (descriptor_set.empty() || identity.world_generation == 0 || identity.scene_id.empty() ||
        identity.lidar_pattern_version != kMid360PatternVersion ||
        identity.lidar_pattern_sha256 != kMid360PatternSha256) {
      throw std::invalid_argument(
          "MCAP session identity must include descriptor, world, scene, and the frozen lidar pattern");
    }

    // 无压缩直写避免在实时 Recorder 内额外引入压缩依赖和不可预测的 CPU 开销。
    mcap::McapWriterOptions options("protobuf");
    options.noChunking = true;
    options.enableDataCRC = true;
    writer.open(output, options);

    mcap::Schema schema("slope_sim.interfaces.v2", "protobuf", descriptor_set);
    writer.addSchema(schema);
    for (const auto& contract : TopicContracts()) {
      mcap::Channel channel(contract.topic, "protobuf", schema.id, {{"type", contract.type_name}});
      writer.addChannel(channel);
      channel_ids.emplace(channel.topic, channel.id);
    }
    const mcap::Metadata manifest{
        "slope_sim.session_manifest",
        {
            {"simulation_session_id", ToHex(identity.simulation_session_id.data(), identity.simulation_session_id.size())},
            {"descriptor_sha256", ToHex(identity.descriptor_sha256.data(), identity.descriptor_sha256.size())},
            {"world_generation", std::to_string(identity.world_generation)},
            {"scene_id", std::move(identity.scene_id)},
            {"lidar_pattern_version", std::move(identity.lidar_pattern_version)},
            {"lidar_pattern_sha256",
             ToHex(identity.lidar_pattern_sha256.data(), identity.lidar_pattern_sha256.size())},
        },
    };
    RequireSuccess(writer.write(manifest), "MCAP manifest write failed");
  }

  AtomicSegment segment;
  AtomicSegmentOutput output;
  mcap::McapWriter writer;
  std::unordered_map<std::string, mcap::ChannelId> channel_ids;
  bool finalized = false;
};

McapSessionWriter::McapSessionWriter(std::filesystem::path final_path,
                                     std::vector<std::byte> descriptor_set,
                                     McapSessionIdentity identity)
    : impl_(std::make_unique<Impl>(std::move(final_path), std::move(descriptor_set), std::move(identity))) {}

McapSessionWriter::~McapSessionWriter() = default;

void McapSessionWriter::Write(std::string_view topic,
                              std::uint32_t sequence,
                              std::uint64_t log_time_ns,
                              std::uint64_t publish_time_ns,
                              const std::vector<std::byte>& payload) {
  if (impl_->finalized) {
    throw std::logic_error("MCAP session is already finalized");
  }
  const auto channel = impl_->channel_ids.find(std::string(topic));
  if (channel == impl_->channel_ids.end()) {
    throw std::invalid_argument("MCAP topic is not in the frozen v2 contract");
  }
  const mcap::Message message{
      channel->second,
      sequence,
      log_time_ns,
      publish_time_ns,
      payload.size(),
      payload.data(),
  };
  RequireSuccess(impl_->writer.write(message), "MCAP message write failed");
}

void McapSessionWriter::Finalize() {
  if (impl_->finalized) {
    throw std::logic_error("MCAP session is already finalized");
  }
  impl_->writer.close();
  impl_->segment.Finalize();
  impl_->finalized = true;
}

}  // namespace slope_sim::client::v2
