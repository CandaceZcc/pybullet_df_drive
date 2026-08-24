// 阶段四 D：从已验证 MCAP 导出逐扫描 PCD/PLY 与会话级 synthetic LVX2。
#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <fcntl.h>
#include <linux/fs.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "../common/sha256.hpp"
#include "slope_sim/client/mcap_session_reader.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

namespace fs = std::filesystem;

struct ExportPlan final {
  fs::path input;
  fs::path descriptor_set;
  fs::path output_dir;
  fs::path result;
};

std::string ReadRegularFile(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("input cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

fs::path ExistingInput(const std::string& raw, const char* name) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument(std::string(name) + " must be an absolute normalized regular file");
  }
  return path;
}

fs::path NewOutputDir(const std::string& raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path()) ||
      fs::exists(path)) {
    throw std::invalid_argument("output-dir must be a new directory below an existing absolute directory");
  }
  return path;
}

fs::path NewResult(const std::string& raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_directory(path.parent_path()) ||
      fs::exists(path)) {
    throw std::invalid_argument("result must be a new file below an existing absolute directory");
  }
  return path;
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

ExportPlan ParsePlan(int argc, char* argv[]) {
  const auto options = ParseOptions(argc, argv);
  if (options.size() != 4 || options.find("--input") == options.end() ||
      options.find("--descriptor-set") == options.end() || options.find("--output-dir") == options.end() ||
      options.find("--result") == options.end()) {
    throw std::invalid_argument("export options are incomplete");
  }
  return {ExistingInput(options.at("--input"), "input"),
          ExistingInput(options.at("--descriptor-set"), "descriptor-set"),
          NewOutputDir(options.at("--output-dir")), NewResult(options.at("--result"))};
}

template <std::size_t Size>
std::string Hex(const std::array<std::byte, Size>& digest) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string output;
  output.reserve(digest.size() * 2);
  for (const std::byte value : digest) {
    const auto byte = std::to_integer<unsigned char>(value);
    output.push_back(kHex[byte >> 4]);
    output.push_back(kHex[byte & 0x0f]);
  }
  return output;
}

void WritePcd(const fs::path& path, const slope_sim::interfaces::v2::LidarPointCloud& cloud) {
  std::ofstream output(path);
  if (!output) throw std::runtime_error("PCD cannot be created");
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "# frame_id " << cloud.frame_id() << "\n"
         << "VERSION 0.7\nFIELDS x y z intensity offset_time_ns line\n"
         << "SIZE 4 4 4 4 4 2\nTYPE F F F F U U\nCOUNT 1 1 1 1 1 1\n"
         << "WIDTH " << cloud.points_size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << cloud.points_size() << "\nDATA ascii\n" << std::setprecision(9);
  for (const auto& point : cloud.points()) {
    output << point.x() << ' ' << point.y() << ' ' << point.z() << ' ' << point.reflectivity() << ' '
           << point.offset_time_ns() << ' ' << point.line() << '\n';
  }
  if (!output) throw std::runtime_error("PCD write failed");
}

void WritePly(const fs::path& path, const slope_sim::interfaces::v2::LidarPointCloud& cloud) {
  std::ofstream output(path);
  if (!output) throw std::runtime_error("PLY cannot be created");
  output << "ply\nformat ascii 1.0\ncomment frame_id " << cloud.frame_id() << "\n"
         << "element vertex " << cloud.points_size() << "\nproperty float x\nproperty float y\n"
         << "property float z\nproperty uint intensity\nproperty uint offset_time_ns\n"
         << "property uchar tag\nproperty uchar line\nend_header\n" << std::setprecision(9);
  for (const auto& point : cloud.points()) {
    output << point.x() << ' ' << point.y() << ' ' << point.z() << ' ' << point.reflectivity() << ' '
           << point.offset_time_ns() << ' ' << point.tag() << ' ' << point.line() << '\n';
  }
  if (!output) throw std::runtime_error("PLY write failed");
}

void WriteU32(std::ostream& output, std::uint32_t value) {
  for (int shift = 0; shift < 32; shift += 8) output.put(static_cast<char>((value >> shift) & 0xff));
}

void WriteU16(std::ostream& output, std::uint16_t value) {
  for (int shift = 0; shift < 16; shift += 8) output.put(static_cast<char>((value >> shift) & 0xff));
}

void WriteU64(std::ostream& output, std::uint64_t value) {
  for (int shift = 0; shift < 64; shift += 8) output.put(static_cast<char>((value >> shift) & 0xff));
}

void WriteI32(std::ostream& output, std::int32_t value) { WriteU32(output, static_cast<std::uint32_t>(value)); }

using LidarCloud = slope_sim::interfaces::v2::LidarPointCloud;
using LidarPoint = slope_sim::interfaces::v2::LidarPoint;

struct ValidatedLidarSession final {
  std::vector<LidarCloud> clouds;
  std::string frame_id;
  std::uint32_t lidar_id = 0;
};

struct Lvx2FramePlan final {
  std::size_t cloud_index;
  int point_begin;
  int point_end;
};

struct Lvx2Stats final {
  std::uint64_t scan_count = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t point_count = 0;
  std::uint64_t package_count = 0;
  std::uint64_t short_tail_package_count = 0;
  std::uint64_t empty_frame_count = 0;
  double max_observed_range_m = 0.0;
};

std::int32_t QuantizeMillimeters(float coordinate) {
  const double millimeters = static_cast<double>(coordinate) * 1000.0;
  if (!std::isfinite(millimeters)) throw std::runtime_error("LiDAR coordinate is not finite");
  const double rounded = std::round(millimeters);
  if (rounded < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
      rounded > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::runtime_error("LiDAR coordinate exceeds LVX2 int32 millimeters");
  }
  return static_cast<std::int32_t>(rounded);
}

template <std::size_t Size>
bool MatchesBytes(std::string_view value, const std::array<std::byte, Size>& expected) {
  return value.size() == expected.size() &&
         std::equal(value.begin(), value.end(), reinterpret_cast<const char*>(expected.data()));
}

ValidatedLidarSession ValidateLidarSession(
    const slope_sim::client::v2::CompletedMcapSession& session) {
  ValidatedLidarSession lidar;
  std::uint64_t previous_sequence = 0;
  std::uint64_t previous_timebase = 0;
  std::uint64_t previous_absolute_point_time = 0;
  bool first_session_point = true;
  bool first_scan = true;
  for (const auto& frame : session.frames) {
    if (frame.topic != "/sim/lidar/points") continue;
    LidarCloud cloud;
    if (frame.payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        !cloud.ParseFromArray(frame.payload.data(), static_cast<int>(frame.payload.size()))) {
      throw std::runtime_error("verified LiDAR payload cannot be decoded");
    }
    if (cloud.sequence() != frame.sequence) {
      throw std::runtime_error("LiDAR payload and MCAP sequence differ");
    }
    if (frame.log_time_ns != cloud.timebase_ns() || frame.publish_time_ns != cloud.timebase_ns()) {
      throw std::runtime_error("LiDAR MCAP metadata differs from payload timebase");
    }
    if (!MatchesBytes(cloud.simulation_session_id(), session.identity.simulation_session_id) ||
        !MatchesBytes(cloud.descriptor_sha256(), session.identity.descriptor_sha256) ||
        cloud.world_generation() != session.identity.world_generation) {
      throw std::runtime_error("LiDAR payload identity differs from MCAP session identity");
    }
    if (cloud.frame_id() != "lidar_link") {
      throw std::runtime_error("LiDAR frame_id is not lidar_link");
    }
    if (!first_scan &&
        (previous_sequence == std::numeric_limits<std::uint64_t>::max() ||
         cloud.sequence() <= previous_sequence || cloud.timebase_ns() <= previous_timebase)) {
      throw std::runtime_error("LiDAR scan sequence or timebase is not strictly continuous");
    }
    if (!first_scan && (cloud.frame_id() != lidar.frame_id || cloud.lidar_id() != lidar.lidar_id)) {
      throw std::runtime_error("LiDAR device identity changed within the MCAP session");
    }

    std::uint32_t previous_offset = 0;
    bool first_point = true;
    for (const auto& point : cloud.points()) {
      if (point.offset_time_ns() >= 100'000'000 ||
          (!first_point && point.offset_time_ns() < previous_offset)) {
        throw std::runtime_error("LiDAR point offsets are outside one ordered 100 ms scan");
      }
      if (cloud.timebase_ns() >
          std::numeric_limits<std::uint64_t>::max() - point.offset_time_ns()) {
        throw std::runtime_error("LiDAR absolute point timestamp overflows uint64");
      }
      const std::uint64_t absolute_point_time = cloud.timebase_ns() + point.offset_time_ns();
      if (!first_session_point && absolute_point_time < previous_absolute_point_time) {
        throw std::runtime_error("LiDAR absolute point timestamps are not ordered across scans");
      }
      (void)QuantizeMillimeters(point.x());
      (void)QuantizeMillimeters(point.y());
      (void)QuantizeMillimeters(point.z());
      if (point.reflectivity() > 255 || point.tag() > 255) {
        throw std::runtime_error("LiDAR reflectivity or tag exceeds LVX2 uint8");
      }
      previous_absolute_point_time = absolute_point_time;
      first_session_point = false;
      previous_offset = point.offset_time_ns();
      first_point = false;
    }

    if (first_scan) {
      lidar.frame_id = cloud.frame_id();
      lidar.lidar_id = cloud.lidar_id();
      first_scan = false;
    }
    previous_sequence = cloud.sequence();
    previous_timebase = cloud.timebase_ns();
    lidar.clouds.push_back(std::move(cloud));
  }
  if (lidar.clouds.empty()) throw std::runtime_error("MCAP session contains no LiDAR scans");
  return lidar;
}

std::vector<Lvx2FramePlan> PlanLvx2Frames(const std::vector<LidarCloud>& clouds) {
  std::vector<Lvx2FramePlan> frames;
  frames.reserve(clouds.size() * 2);
  for (std::size_t cloud_index = 0; cloud_index < clouds.size(); ++cloud_index) {
    const auto& cloud = clouds[cloud_index];
    int split = 0;
    while (split < cloud.points_size() && cloud.points(split).offset_time_ns() < 50'000'000) ++split;
    frames.push_back({cloud_index, 0, split});
    frames.push_back({cloud_index, split, cloud.points_size()});
  }
  return frames;
}

Lvx2Stats SummarizeLvx2(const std::vector<LidarCloud>& clouds,
                        const std::vector<Lvx2FramePlan>& frames) {
  Lvx2Stats stats;
  stats.scan_count = clouds.size();
  stats.frame_count = frames.size();
  // 记录实际导出的最远点，供 1.5× 量程验收审计，不能把配置上限伪装成采集结果。
  for (const auto& cloud : clouds) {
    for (const auto& point : cloud.points()) {
      stats.max_observed_range_m = std::max(
          stats.max_observed_range_m,
          std::hypot(std::hypot(static_cast<double>(point.x()), static_cast<double>(point.y())),
                     static_cast<double>(point.z())));
    }
  }
  for (const auto& frame : frames) {
    const std::uint64_t point_count = static_cast<std::uint64_t>(frame.point_end - frame.point_begin);
    stats.point_count += point_count;
    if (point_count == 0) {
      ++stats.empty_frame_count;
      continue;
    }
    stats.package_count += (point_count + 95) / 96;
    if (point_count % 96 != 0) ++stats.short_tail_package_count;
  }
  return stats;
}

std::uint64_t Lvx2FrameSize(const Lvx2FramePlan& frame) {
  const std::uint64_t point_count = static_cast<std::uint64_t>(frame.point_end - frame.point_begin);
  const std::uint64_t package_count = (point_count + 95) / 96;
  return 24 + package_count * 27 + point_count * 14;
}

/// 标准布局仍承载 synthetic 点云；MCAP 才是保留完整点字段的权威记录。
void WriteSyntheticLvx2(const fs::path& path,
                        const ValidatedLidarSession& lidar,
                        const std::vector<Lvx2FramePlan>& frames) {
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("synthetic LVX2 cannot be created");

  std::array<char, 16> signature{};
  constexpr std::string_view kSignature = "livox_tech";
  std::copy(kSignature.begin(), kSignature.end(), signature.begin());
  output.write(signature.data(), signature.size());
  output.put('\x02');
  output.put('\0');
  output.put('\0');
  output.put('\0');
  WriteU32(output, 0xAC0EA767);
  WriteU32(output, 50);
  output.put('\x01');

  constexpr std::string_view kSyntheticSn = "SLOPESIM00000001";
  output.write(kSyntheticSn.data(), kSyntheticSn.size());
  const std::array<char, 16> empty_hub_sn{};
  output.write(empty_hub_sn.data(), empty_hub_sn.size());
  WriteU32(output, lidar.lidar_id);
  output.put('\0');
  output.put('\x09');
  output.put('\0');
  const std::array<char, 24> zero_extrinsics{};
  output.write(zero_extrinsics.data(), zero_extrinsics.size());

  std::vector<std::uint64_t> offsets(frames.size() + 1, 92);
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const std::uint64_t size = Lvx2FrameSize(frames[index]);
    if (offsets[index] > std::numeric_limits<std::uint64_t>::max() - size) {
      throw std::runtime_error("synthetic LVX2 file size overflows uint64");
    }
    offsets[index + 1] = offsets[index] + size;
  }

  std::uint16_t udp_counter = 0;
  for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    const auto& frame = frames[frame_index];
    const auto& cloud = lidar.clouds[frame.cloud_index];
    WriteU64(output, offsets[frame_index]);
    WriteU64(output, offsets[frame_index + 1]);
    WriteU64(output, frame_index);

    for (int begin = frame.point_begin; begin < frame.point_end; begin += 96) {
      const int end = std::min(begin + 96, frame.point_end);
      const LidarPoint& first = cloud.points(begin);
      output.put('\0');
      WriteU32(output, lidar.lidar_id);
      output.put('\x08');
      output.put('\0');
      WriteU64(output, cloud.timebase_ns() + first.offset_time_ns());
      WriteU16(output, udp_counter++);
      output.put('\x01');
      WriteU32(output, static_cast<std::uint32_t>((end - begin) * 14));
      output.put(static_cast<char>(frame_index & 0xff));
      WriteU32(output, 0);
      for (int point_index = begin; point_index < end; ++point_index) {
        const auto& point = cloud.points(point_index);
        WriteI32(output, QuantizeMillimeters(point.x()));
        WriteI32(output, QuantizeMillimeters(point.y()));
        WriteI32(output, QuantizeMillimeters(point.z()));
        output.put(static_cast<char>(point.reflectivity()));
        output.put(static_cast<char>(point.tag()));
      }
    }
  }
  if (!output) throw std::runtime_error("synthetic LVX2 write failed");
}

std::string JsonString(std::string_view value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20) {
          output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

void WriteSidecar(const fs::path& path,
                  const ValidatedLidarSession& lidar,
                  const slope_sim::client::v2::McapSessionIdentity& identity,
                  std::string_view source_hash,
                  const Lvx2Stats& stats) {
  std::ofstream output(path);
  if (!output) throw std::runtime_error("synthetic LVX2 sidecar cannot be created");
  output << "{\"synthetic\":true,\"format\":\"LVX2\",\"version\":\"2.0.0.0\","
         << "\"source_mcap_sha256\":\"" << source_hash << "\","
         << "\"simulation_session_id\":\"" << Hex(identity.simulation_session_id) << "\","
         << "\"descriptor_sha256\":\"" << Hex(identity.descriptor_sha256) << "\","
         << "\"lidar_pattern_version\":" << JsonString(identity.lidar_pattern_version) << ','
         << "\"lidar_pattern_sha256\":\"" << Hex(identity.lidar_pattern_sha256) << "\","
         << "\"world_generation\":" << identity.world_generation
         << ",\"scene\":" << JsonString(identity.scene_id)
         << ",\"frame_id\":" << JsonString(lidar.frame_id)
         << ",\"lidar_sn\":\"SLOPESIM00000001\",\"hub_sn\":\"\","
         << "\"lidar_id\":" << lidar.lidar_id
         << ",\"lidar_type\":0,\"device_type\":9,\"extrinsic_enabled\":false,"
         << "\"extrinsics\":{\"roll_deg\":0,\"pitch_deg\":0,\"yaw_deg\":0,"
            "\"x_m\":0,\"y_m\":0,\"z_m\":0},"
         << "\"quantization_m\":0.001,\"frame_duration_ms\":50,"
         << "\"max_points_per_package\":96,"
         << "\"package_timestamp_rule\":\"first_point_absolute_ns\","
         << "\"scan_count\":" << stats.scan_count << ",\"frame_count\":" << stats.frame_count
         << ",\"point_count\":" << stats.point_count << ",\"package_count\":" << stats.package_count
         << ",\"max_observed_range_m\":" << stats.max_observed_range_m
         << ",\"short_tail_package_count\":" << stats.short_tail_package_count
         << ",\"empty_frame_count\":" << stats.empty_frame_count
         << ",\"padding_points\":0,\"dropped_points\":0,"
         << "\"line_loss\":true,\"point_offset_loss\":true,"
         << "\"line_preserved_in_pcd_ply_and_mcap\":true,"
         << "\"point_offset_time_ns_preserved_in_pcd_ply_and_mcap\":true}\n";
  if (!output) throw std::runtime_error("synthetic LVX2 sidecar write failed");
}

void WriteResult(const fs::path& path, const Lvx2Stats& stats) {
  int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) throw std::runtime_error("export result cannot be created exclusively");
  try {
    const std::string value = "{\"clean_shutdown\":true,\"lidar_frames\":" +
        std::to_string(stats.scan_count) + ",\"lvx2\":\"lidar.lvx2\",\"lvx2_frames\":" +
        std::to_string(stats.frame_count) + ",\"lvx2_packages\":" +
        std::to_string(stats.package_count) + ",\"lvx2_points\":" +
        std::to_string(stats.point_count) + ",\"role\":\"export\"}\n";
    std::size_t written = 0;
    while (written < value.size()) {
      const ssize_t count = ::write(descriptor, value.data() + written, value.size() - written);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("export result write failed");
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) throw std::runtime_error("export result sync failed");
    const int closing_descriptor = descriptor;
    descriptor = -1;
    if (::close(closing_descriptor) != 0) throw std::runtime_error("export result sync failed");
  } catch (...) {
    if (descriptor >= 0) (void)::close(descriptor);
    std::error_code ignored;
    fs::remove(path, ignored);
    throw;
  }
}

struct DirectoryIdentity final {
  dev_t device = 0;
  ino_t inode = 0;
};

DirectoryIdentity DirectoryIdentityOf(const fs::path& path) {
  struct stat status {};
  if (::lstat(path.c_str(), &status) != 0 || !S_ISDIR(status.st_mode)) {
    throw std::runtime_error("export directory identity cannot be read");
  }
  return {status.st_dev, status.st_ino};
}

bool StillOwnsDirectory(const fs::path& path, const DirectoryIdentity& expected) {
  struct stat status {};
  return ::lstat(path.c_str(), &status) == 0 && S_ISDIR(status.st_mode) &&
         status.st_dev == expected.device && status.st_ino == expected.inode;
}

bool RenameNoReplace(const fs::path& source, const fs::path& target) {
  return ::syscall(SYS_renameat2, AT_FDCWD, source.c_str(), AT_FDCWD, target.c_str(),
                   RENAME_NOREPLACE) == 0;
}

void PublishOutputDirectory(const fs::path& staging, const fs::path& output_dir) {
  if (RenameNoReplace(staging, output_dir)) return;
  if (errno == EEXIST) throw std::runtime_error("export output directory already exists at publication");
  throw std::runtime_error("export output directory cannot be published");
}

void ReserveOutputForTest(const fs::path& output_dir) {
  const char* reserved = std::getenv("STAGE4_EXPORT_TEST_RESERVE_OUTPUT_DIR");
  if (reserved == nullptr || output_dir != fs::path(reserved)) return;
  if (!fs::create_directory(output_dir)) throw std::runtime_error("test output reservation cannot be created");
}

void ReserveResultForRollbackTest(const fs::path& output_dir, const fs::path& result) {
  const char* target = std::getenv("STAGE4_EXPORT_TEST_REPLACE_OUTPUT_DURING_ROLLBACK");
  if (target == nullptr || output_dir != fs::path(target)) return;
  std::ofstream competitor(result);
  competitor << "competitor result\n";
  if (!competitor) throw std::runtime_error("test result reservation cannot be written");
}

void ReplaceOutputAfterOwnershipCheckForTest(const fs::path& output_dir) {
  const char* target = std::getenv("STAGE4_EXPORT_TEST_REPLACE_OUTPUT_DURING_ROLLBACK");
  if (target == nullptr || output_dir != fs::path(target)) return;
  fs::rename(output_dir, output_dir.string() + ".displaced-for-test");
  if (!fs::create_directory(output_dir)) throw std::runtime_error("test competitor output cannot be created");
  std::ofstream marker(output_dir / "competitor.txt");
  marker << "competitor\n";
  if (!marker) throw std::runtime_error("test competitor output cannot be written");
}

void RollbackPublishedOutput(const fs::path& output_dir,
                             const DirectoryIdentity& expected) {
  if (StillOwnsDirectory(output_dir, expected)) {
    ReplaceOutputAfterOwnershipCheckForTest(output_dir);
  }
  const fs::path quarantine = output_dir.string() + ".rollback-" + std::to_string(::getpid());
  if (!RenameNoReplace(output_dir, quarantine)) return;
  if (StillOwnsDirectory(quarantine, expected)) {
    std::error_code ignored;
    fs::remove_all(quarantine, ignored);
    return;
  }
  (void)RenameNoReplace(quarantine, output_dir);
}

int RunExport(const ExportPlan& plan) {
  const std::string descriptor = ReadRegularFile(plan.descriptor_set);
  const auto session = slope_sim::client::v2::ReadCompletedMcapSession(plan.input);
  const std::string manifest_digest(reinterpret_cast<const char*>(session.identity.descriptor_sha256.data()),
                                    session.identity.descriptor_sha256.size());
  if (stage4::Bytes(stage4::Sha256(descriptor)) != manifest_digest) {
    throw std::runtime_error("supplied descriptor differs from MCAP session identity");
  }
  const ValidatedLidarSession lidar = ValidateLidarSession(session);
  const std::vector<Lvx2FramePlan> lvx2_frames = PlanLvx2Frames(lidar.clouds);
  const Lvx2Stats lvx2_stats = SummarizeLvx2(lidar.clouds, lvx2_frames);
  if (lvx2_stats.package_count >
      static_cast<std::uint64_t>(std::numeric_limits<std::uint16_t>::max()) + 1) {
    throw std::runtime_error("LVX2 UDP counter overflows uint16");
  }
  const fs::path staging = plan.output_dir.string() + ".partial-" + std::to_string(::getpid());
  const fs::path result_staging = plan.result.string() + ".partial-" + std::to_string(::getpid());
  if (!fs::create_directory(staging)) throw std::runtime_error("export staging directory cannot be created");
  const DirectoryIdentity staging_identity = DirectoryIdentityOf(staging);
  bool output_published = false;
  bool result_staged = false;
  bool result_published = false;
  try {
    const std::string source_hash = Hex(stage4::Sha256(ReadRegularFile(plan.input)));
    for (const auto& cloud : lidar.clouds) {
      const std::string base = "lidar-" + [&cloud] { std::ostringstream value; value << std::setw(10)
          << std::setfill('0') << cloud.sequence(); return value.str(); }();
      const fs::path target = staging / base;
      WritePcd(target.string() + ".pcd", cloud);
      WritePly(target.string() + ".ply", cloud);
    }
    WriteSyntheticLvx2(staging / "lidar.lvx2", lidar, lvx2_frames);
    WriteSidecar(staging / "lidar.lvx2.json", lidar, session.identity, source_hash, lvx2_stats);
    WriteResult(result_staging, lvx2_stats);
    result_staged = true;
    ReserveOutputForTest(plan.output_dir);
    PublishOutputDirectory(staging, plan.output_dir);
    output_published = true;
    ReserveResultForRollbackTest(plan.output_dir, plan.result);
    if (::link(result_staging.c_str(), plan.result.c_str()) != 0) {
      throw std::runtime_error("export result cannot be published exclusively");
    }
    result_published = true;
    if (::unlink(result_staging.c_str()) != 0) throw std::runtime_error("export result staging cleanup failed");
    return 0;
  } catch (...) {
    std::error_code ignored;
    fs::remove_all(staging, ignored);
    if (result_staged) fs::remove(result_staging, ignored);
    if (result_published) fs::remove(plan.result, ignored);
    if (output_published) RollbackPublishedOutput(plan.output_dir, staging_identity);
    throw;
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    return RunExport(ParsePlan(argc, argv));
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
