// 阶段四 D：Export 必须从完成 MCAP 导出可回读 PCD/PLY 与标准会话级 synthetic LVX2。
#include <array>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <fcntl.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../../common/sha256.hpp"
#include "slope_sim/client/mcap_session_writer.hpp"

namespace {

namespace fs = std::filesystem;

constexpr std::string_view kSessionId(
    "\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb\xcc\xdd\xee\xff", 16);

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

std::vector<std::byte> ReadFixture(const std::string& relative_path) {
  const auto root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  std::ifstream input(root / relative_path, std::ios::binary);
  Require(static_cast<bool>(input), "fixture cannot be opened");
  const std::string raw{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  return {reinterpret_cast<const std::byte*>(raw.data()),
          reinterpret_cast<const std::byte*>(raw.data()) + raw.size()};
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  Require(static_cast<bool>(input), "export output cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
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

std::uint32_t ReadU32(std::string_view bytes, std::size_t offset) {
  Require(offset + 4 <= bytes.size(), "LVX2 uint32 is truncated");
  std::uint32_t value = 0;
  for (int shift = 0; shift < 32; shift += 8) {
    value |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset + shift / 8])) << shift;
  }
  return value;
}

std::uint16_t ReadU16(std::string_view bytes, std::size_t offset) {
  Require(offset + 2 <= bytes.size(), "LVX2 uint16 is truncated");
  return static_cast<std::uint16_t>(static_cast<unsigned char>(bytes[offset])) |
         static_cast<std::uint16_t>(static_cast<unsigned char>(bytes[offset + 1]) << 8);
}

std::uint64_t ReadU64(std::string_view bytes, std::size_t offset) {
  Require(offset + 8 <= bytes.size(), "LVX2 uint64 is truncated");
  std::uint64_t value = 0;
  for (int shift = 0; shift < 64; shift += 8) {
    value |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[offset + shift / 8])) << shift;
  }
  return value;
}

std::int32_t ReadI32(std::string_view bytes, std::size_t offset) {
  return static_cast<std::int32_t>(ReadU32(bytes, offset));
}

void AppendVarint(std::string& bytes, std::uint64_t value) {
  while (value >= 0x80) {
    bytes.push_back(static_cast<char>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  bytes.push_back(static_cast<char>(value));
}

void AppendFixed32(std::string& bytes, float value) {
  std::uint32_t encoded = 0;
  static_assert(sizeof(encoded) == sizeof(value));
  std::memcpy(&encoded, &value, sizeof(encoded));
  for (int shift = 0; shift < 32; shift += 8) {
    bytes.push_back(static_cast<char>((encoded >> shift) & 0xff));
  }
}

void AppendBytes(std::string& bytes, std::uint32_t field, std::string_view value) {
  AppendVarint(bytes, (field << 3) | 2);
  AppendVarint(bytes, value.size());
  bytes.append(value);
}

void AppendUInt(std::string& bytes, std::uint32_t field, std::uint64_t value) {
  AppendVarint(bytes, field << 3);
  AppendVarint(bytes, value);
}

struct TestPoint final {
  std::uint32_t offset_time_ns;
  float x;
  float y;
  float z;
  std::uint32_t reflectivity;
  std::uint32_t tag;
};

std::vector<std::byte> MakeLidarPayload(std::uint32_t sequence,
                                        std::uint64_t timebase_ns,
                                        std::uint32_t lidar_id,
                                        const std::vector<TestPoint>& points,
                                        const std::array<std::byte, 32>& descriptor_sha256,
                                        std::string_view frame_id = "lidar_link",
                                        std::uint32_t world_generation = 7,
                                        std::string_view session_id = kSessionId) {
  // 测试内直接编码冻结 v2 wire 字段，避免为测试新增生产 include/link 接线。
  std::string bytes;
  AppendUInt(bytes, 1, timebase_ns);
  AppendBytes(bytes, 2, frame_id);
  AppendUInt(bytes, 3, points.size());
  AppendUInt(bytes, 4, lidar_id);
  for (const auto& value : points) {
    std::string point;
    AppendUInt(point, 1, value.offset_time_ns);
    AppendVarint(point, (2 << 3) | 5);
    AppendFixed32(point, value.x);
    AppendVarint(point, (3 << 3) | 5);
    AppendFixed32(point, value.y);
    AppendVarint(point, (4 << 3) | 5);
    AppendFixed32(point, value.z);
    AppendUInt(point, 5, value.reflectivity);
    AppendUInt(point, 6, value.tag);
    AppendUInt(point, 7, 3);
    AppendBytes(bytes, 5, point);
  }
  AppendUInt(bytes, 6, sequence);
  AppendUInt(bytes, 7, world_generation);
  AppendBytes(bytes, 8, session_id);
  AppendBytes(bytes, 9,
              std::string_view(reinterpret_cast<const char*>(descriptor_sha256.data()), descriptor_sha256.size()));
  return {reinterpret_cast<const std::byte*>(bytes.data()),
          reinterpret_cast<const std::byte*>(bytes.data()) + bytes.size()};
}

struct ExportRun final {
  int exit_code;
  std::string error;
};

ExportRun RunExport(const fs::path& root,
                    const fs::path& input_path,
                    const fs::path& output_dir,
                    const fs::path& result_path,
                    std::uint64_t file_size_limit = 0,
                    bool reserve_output_before_publish = false,
                    bool replace_output_during_rollback = false) {
  const fs::path error_path = output_dir.string() + ".stderr";
  const pid_t child = ::fork();
  Require(child >= 0, "fork failed");
  if (child == 0) {
    const std::string executable = STAGE4_EXPORT_EXECUTABLE;
    const std::string input = input_path.string();
    const std::string descriptor_arg = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").string();
    const std::string output_arg = output_dir.string();
    const std::string result_arg = result_path.string();
    const int error_descriptor = ::open(error_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (error_descriptor < 0 || ::dup2(error_descriptor, STDERR_FILENO) < 0) _exit(126);
    (void)::close(error_descriptor);
    if (file_size_limit != 0) {
      const rlimit limit{file_size_limit, file_size_limit};
      if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) _exit(126);
      (void)::signal(SIGXFSZ, SIG_IGN);
    }
    if (reserve_output_before_publish) {
      Require(::setenv("STAGE4_EXPORT_TEST_RESERVE_OUTPUT_DIR", output_arg.c_str(), 1) == 0,
              "cannot configure output reservation fixture");
    }
    if (replace_output_during_rollback) {
      Require(::setenv("STAGE4_EXPORT_TEST_REPLACE_OUTPUT_DURING_ROLLBACK", output_arg.c_str(), 1) == 0,
              "cannot configure rollback replacement fixture");
    }
    char* const args[] = {
        const_cast<char*>(executable.c_str()), const_cast<char*>("--input"),
        const_cast<char*>(input.c_str()), const_cast<char*>("--descriptor-set"),
        const_cast<char*>(descriptor_arg.c_str()), const_cast<char*>("--output-dir"),
        const_cast<char*>(output_arg.c_str()), const_cast<char*>("--result"),
        const_cast<char*>(result_arg.c_str()), nullptr};
    ::execv(args[0], args);
    _exit(127);
  }
  int status = 0;
  Require(::waitpid(child, &status, 0) == child && WIFEXITED(status),
          "export participant did not exit normally");
  const std::string error = ReadText(error_path);
  fs::remove(error_path);
  return ExportRun{WEXITSTATUS(status), error};
}

void RequireRejectedAndPreserved(const fs::path& root,
                                const fs::path& recording,
                                const fs::path& output,
                                const fs::path& result,
                                std::string_view expected_error,
                                const char* message) {
  const std::string source_before = ReadText(recording);
  const ExportRun run = RunExport(root, recording, output, result);
  const bool rejected = run.exit_code != 0 && run.error.find(expected_error) != std::string::npos &&
                        !fs::exists(output) && !fs::exists(result) && ReadText(recording) == source_before;
  if (!rejected) {
    std::cerr << "expected export error: " << expected_error << "; actual: " << run.error;
  }
  Require(rejected, message);
}

void WriteCrossReadFixture(const fs::path& root, const fs::path& directory) {
  using slope_sim::client::v2::McapSessionIdentity;
  using slope_sim::client::v2::McapSessionWriter;

  Require(fs::is_directory(directory) && fs::is_empty(directory),
          "cross-read fixture directory must exist and be empty");
  const auto descriptor_bytes = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  const std::string descriptor(reinterpret_cast<const char*>(descriptor_bytes.data()), descriptor_bytes.size());
  const McapSessionIdentity identity{
      {std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
       std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
       std::byte{0x88}, std::byte{0x99}, std::byte{0xaa}, std::byte{0xbb},
       std::byte{0xcc}, std::byte{0xdd}, std::byte{0xee}, std::byte{0xff}},
      stage4::Sha256(descriptor),
      7,
      "cross-read-fixture",
      std::string(slope_sim::client::v2::kMid360PatternVersion),
      slope_sim::client::v2::kMid360PatternSha256,
  };
  const std::vector<TestPoint> points{
      {1, 1.2344F, -2.3456F, 3.4566F, 77, 2},
      {2, 0.0006F, -0.0006F, 0.0F, 1, 3},
      {49'999'999, -1.0006F, 2.0006F, -3.0006F, 255, 255},
      {50'000'000, 4.4444F, 5.5556F, -6.6666F, 8, 9},
      {50'000'001, -7.7777F, 8.8888F, 9.9999F, 10, 11},
  };
  const fs::path recording = directory / "fixture.mcap";
  {
    McapSessionWriter writer(recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(41, 5'000'000'000, 42, points, identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 41, 5'000'000'000, 5'000'000'000, payload);
    writer.Finalize();
  }
  const fs::path output = directory / "export";
  const fs::path result = directory / "result.json";
  Require(RunExport(root, recording, output, result).exit_code == 0,
          "cross-read fixture export did not exit cleanly");
  std::ofstream expected(directory / "expected.json");
  Require(static_cast<bool>(expected), "cross-read expected JSON cannot be created");
  expected << "{\"lvx2\":\"export/lidar.lvx2\",\"frame_count\":2,\"package_count\":2,\"points\":["
           << "{\"x_m\":1.2344,\"y_m\":-2.3456,\"z_m\":3.4566,\"reflectivity\":77,\"tag\":2,\"package_timestamp_ns\":5000000001},"
           << "{\"x_m\":0.0006,\"y_m\":-0.0006,\"z_m\":0.0,\"reflectivity\":1,\"tag\":3,\"package_timestamp_ns\":5000000001},"
           << "{\"x_m\":-1.0006,\"y_m\":2.0006,\"z_m\":-3.0006,\"reflectivity\":255,\"tag\":255,\"package_timestamp_ns\":5000000001},"
           << "{\"x_m\":4.4444,\"y_m\":5.5556,\"z_m\":-6.6666,\"reflectivity\":8,\"tag\":9,\"package_timestamp_ns\":5050000000},"
           << "{\"x_m\":-7.7777,\"y_m\":8.8888,\"z_m\":9.9999,\"reflectivity\":10,\"tag\":11,\"package_timestamp_ns\":5050000000}] }\n";
  Require(static_cast<bool>(expected), "cross-read expected JSON write failed");
}

}  // namespace

int main(int argc, char* argv[]) {
  using slope_sim::client::v2::McapSessionIdentity;
  using slope_sim::client::v2::McapSessionWriter;

  const fs::path root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  if (argc == 3 && std::string_view(argv[1]) == "--cross-read-fixture") {
    WriteCrossReadFixture(root, argv[2]);
    return 0;
  }
  Require(argc == 1, "unexpected stage4 export test arguments");
  const fs::path directory = fs::temp_directory_path() / ("slope-sim-export-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const fs::path recording = directory / "session.mcap";
  const auto descriptor_bytes = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  const std::string descriptor(reinterpret_cast<const char*>(descriptor_bytes.data()), descriptor_bytes.size());
  const McapSessionIdentity identity{
      {std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
       std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
       std::byte{0x88}, std::byte{0x99}, std::byte{0xaa}, std::byte{0xbb},
       std::byte{0xcc}, std::byte{0xdd}, std::byte{0xee}, std::byte{0xff}},
      stage4::Sha256(descriptor),
      7,
      "export-integration-scene",
      std::string(slope_sim::client::v2::kMid360PatternVersion),
      slope_sim::client::v2::kMid360PatternSha256,
  };
  {
    McapSessionWriter writer(recording, descriptor_bytes, identity);
    std::vector<TestPoint> first_points;
    first_points.push_back({10'000'000, 1.2344F, -2.3456F, 3.4566F, 77, 2});
    for (std::uint32_t index = 1; index < 97; ++index) {
      first_points.push_back({10'000'000 + index, static_cast<float>(index) / 1000.0F,
                              -static_cast<float>(index) / 1000.0F, 0.0F, index, index % 4});
    }
    const auto first = MakeLidarPayload(2, 1'000'000'000, 42, first_points, identity.descriptor_sha256);
    const auto second = MakeLidarPayload(
        3, 1'100'000'000, 42, {{60'000'000, -4.5674F, 5.6785F, -6.7896F, 88, 3}},
        identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 2, 1'000'000'000, 1'000'000'000, first);
    writer.Write("/sim/lidar/points", 3, 1'100'000'000, 1'100'000'000, second);
    writer.Finalize();
  }
  const std::string source_before = ReadText(recording);

  const fs::path output = directory / "export";
  const fs::path result = directory / "export.json";
  Require(RunExport(root, recording, output, result).exit_code == 0,
          "export participant did not exit cleanly");

  const fs::path base = output / "lidar-0000000002";
  const std::string pcd = ReadText(base.string() + ".pcd");
  Require(pcd.find("FIELDS x y z intensity offset_time_ns line\n") != std::string::npos &&
              pcd.find("POINTS 97\nDATA ascii\n") != std::string::npos,
          "PCD header is not readable or has the wrong point count");
  const std::string ply = ReadText(base.string() + ".ply");
  Require(ply.find("ply\nformat ascii 1.0\n") == 0 &&
              ply.find("element vertex 97\n") != std::string::npos &&
              ply.find("end_header\n") != std::string::npos,
          "PLY header is not readable or has the wrong point count");
  const std::string lvx2 = ReadText(output / "lidar.lvx2");
  Require(lvx2.size() >= 92 && lvx2.substr(0, 10) == "livox_tech" &&
              lvx2.substr(10, 6) == std::string(6, '\0') &&
              static_cast<unsigned char>(lvx2[16]) == 2 && static_cast<unsigned char>(lvx2[17]) == 0 &&
              static_cast<unsigned char>(lvx2[18]) == 0 && static_cast<unsigned char>(lvx2[19]) == 0 &&
              ReadU32(lvx2, 20) == 0xAC0EA767 && ReadU32(lvx2, 24) == 50 &&
              static_cast<unsigned char>(lvx2[28]) == 1,
          "LVX2 public/private header is not standard v2.0.0.0");
  Require(lvx2.substr(29, 16) == "SLOPESIM00000001" && lvx2.substr(45, 16) == std::string(16, '\0') &&
              ReadU32(lvx2, 61) == 42 && static_cast<unsigned char>(lvx2[65]) == 0 &&
              static_cast<unsigned char>(lvx2[66]) == 9 && static_cast<unsigned char>(lvx2[67]) == 0,
          "LVX2 synthetic Mid-360 device metadata is wrong");
  std::size_t offset = 92;
  const std::array<std::vector<std::uint32_t>, 4> frame_packages{{{96, 1}, {}, {}, {1}}};
  const std::array<std::vector<std::uint64_t>, 4> package_timestamps{{
      {1'010'000'000, 1'010'000'096}, {}, {}, {1'160'000'000}}};
  std::uint16_t udp_counter = 0;
  for (std::uint64_t index = 0; index < 4; ++index) {
    Require(ReadU64(lvx2, offset) == offset && ReadU64(lvx2, offset + 16) == index,
            "LVX2 frame offsets or indices are not contiguous");
    const std::uint64_t next = ReadU64(lvx2, offset + 8);
    if (frame_packages[index].empty()) {
      Require(next == offset + 24, "empty 50 ms half must retain an empty LVX2 frame");
    } else {
      std::size_t package_offset = offset + 24;
      for (std::size_t package_index = 0; package_index < frame_packages[index].size(); ++package_index) {
        const std::uint32_t point_count = frame_packages[index][package_index];
        Require(static_cast<unsigned char>(lvx2[package_offset]) == 0 &&
                    ReadU32(lvx2, package_offset + 1) == 42 &&
                    static_cast<unsigned char>(lvx2[package_offset + 5]) == 8 &&
                    static_cast<unsigned char>(lvx2[package_offset + 6]) == 0 &&
                    ReadU64(lvx2, package_offset + 7) == package_timestamps[index][package_index] &&
                    ReadU16(lvx2, package_offset + 15) == udp_counter++ &&
                    static_cast<unsigned char>(lvx2[package_offset + 17]) == 1 &&
                    ReadU32(lvx2, package_offset + 18) == point_count * 14 &&
                    static_cast<unsigned char>(lvx2[package_offset + 22]) == static_cast<unsigned char>(index) &&
                    lvx2.substr(package_offset + 23, 4) == std::string(4, '\0'),
                "LVX2 package header, timestamp, or short length is wrong");
        package_offset += 27 + point_count * 14;
      }
      Require(next == package_offset, "LVX2 frame next offset does not match its packages");
    }
    offset = static_cast<std::size_t>(next);
  }
  Require(offset == lvx2.size() && ReadI32(lvx2, 92 + 24 + 27) == 1234 &&
              ReadI32(lvx2, 92 + 24 + 27 + 4) == -2346 &&
              ReadI32(lvx2, 92 + 24 + 27 + 8) == 3457 &&
              static_cast<unsigned char>(lvx2[92 + 24 + 27 + 12]) == 77 &&
              static_cast<unsigned char>(lvx2[92 + 24 + 27 + 13]) == 2 &&
              ReadI32(lvx2, 92 + 24 + 27 + 96 * 14 + 27) == 96,
          "LVX2 final next offset or millimeter quantization is wrong");
  const std::string sidecar = ReadText(output / "lidar.lvx2.json");
  const std::string source_hash = Hex(stage4::Sha256(source_before));
  Require(sidecar.find("\"synthetic\":true") != std::string::npos &&
              sidecar.find("\"format\":\"LVX2\"") != std::string::npos &&
              sidecar.find("\"version\":\"2.0.0.0\"") != std::string::npos &&
              sidecar.find("\"source_mcap_sha256\":\"" + source_hash + "\"") != std::string::npos &&
              sidecar.find("\"simulation_session_id\":\"00112233445566778899aabbccddeeff\"") !=
                  std::string::npos &&
              sidecar.find("\"descriptor_sha256\":\"" + Hex(identity.descriptor_sha256) + "\"") !=
                  std::string::npos &&
              sidecar.find("\"lidar_pattern_version\":\"livox-mid360-800000-v1\"") !=
                  std::string::npos &&
              sidecar.find("\"lidar_pattern_sha256\":\"4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca\"") !=
                  std::string::npos &&
              sidecar.find("\"world_generation\":7") != std::string::npos &&
              sidecar.find("\"scene\":\"export-integration-scene\"") != std::string::npos &&
              sidecar.find("\"frame_id\":\"lidar_link\"") != std::string::npos &&
              sidecar.find("\"lidar_sn\":\"SLOPESIM00000001\"") != std::string::npos &&
              sidecar.find("\"lidar_id\":42") != std::string::npos &&
              sidecar.find("\"device_type\":9") != std::string::npos &&
              sidecar.find("\"extrinsic_enabled\":false") != std::string::npos &&
              sidecar.find("\"quantization_m\":0.001") != std::string::npos &&
              sidecar.find("\"frame_duration_ms\":50") != std::string::npos &&
              sidecar.find("\"package_timestamp_rule\":\"first_point_absolute_ns\"") !=
                  std::string::npos &&
              sidecar.find("\"padding_points\":0") != std::string::npos &&
              sidecar.find("\"dropped_points\":0") != std::string::npos &&
              sidecar.find("\"line_loss\":true") != std::string::npos &&
              sidecar.find("\"point_offset_loss\":true") != std::string::npos &&
              sidecar.find("\"scan_count\":2") != std::string::npos &&
              sidecar.find("\"frame_count\":4") != std::string::npos &&
              sidecar.find("\"point_count\":98") != std::string::npos &&
              sidecar.find("\"max_observed_range_m\":9.96018") != std::string::npos &&
              sidecar.find("\"package_count\":3") != std::string::npos &&
              sidecar.find("\"short_tail_package_count\":2") != std::string::npos &&
              sidecar.find("\"empty_frame_count\":2") != std::string::npos,
          "LVX2 sidecar does not disclose synthetic lossiness");
  Require(ReadText(result) ==
              "{\"clean_shutdown\":true,\"lidar_frames\":2,\"lvx2\":\"lidar.lvx2\","
              "\"lvx2_frames\":4,\"lvx2_packages\":3,\"lvx2_points\":98,\"role\":\"export\"}\n",
          "export result is incomplete");
  const fs::path retry_output = directory / "export-retry";
  const fs::path retry_result = directory / "export-retry.json";
  Require(RunExport(root, recording, retry_output, retry_result).exit_code == 0,
          "export retry did not exit cleanly");
  Require(fs::is_regular_file(retry_output / "lidar-0000000002.pcd") &&
              ReadText(retry_output / "lidar.lvx2") == lvx2 &&
              ReadText(recording) == source_before,
          "export retry changed the authoritative MCAP session");

  const fs::path failed_output = directory / "export-result-failure";
  const fs::path unwritable_result =
      fs::path("/proc") / ("slope-sim-export-result-" + std::to_string(::getpid()) + ".json");
  Require(!fs::exists(unwritable_result), "result failure fixture unexpectedly exists");
  Require(RunExport(root, recording, failed_output, unwritable_result).exit_code != 0 && !fs::exists(failed_output) &&
              !fs::exists(unwritable_result) && ReadText(recording) == source_before,
          "result failure published partial output or changed the authoritative MCAP session");

  const fs::path competing_output = directory / "export-competing-output";
  const fs::path competing_result = directory / "export-competing-output.json";
  const ExportRun competing_run = RunExport(
      root, recording, competing_output, competing_result, 0, true);
  Require(competing_run.exit_code != 0 && fs::is_directory(competing_output) &&
              fs::is_empty(competing_output) &&
              !fs::exists(competing_result) && ReadText(recording) == source_before,
          "export replaced or populated a competing empty output directory during publication");

  const fs::path rollback_output = directory / "export-rollback-race";
  const fs::path rollback_result = directory / "export-rollback-race.json";
  const ExportRun rollback_run = RunExport(
      root, recording, rollback_output, rollback_result, 0, false, true);
  Require(rollback_run.exit_code != 0 && fs::is_directory(rollback_output) &&
              ReadText(rollback_output / "competitor.txt") == "competitor\n" &&
              !fs::exists(rollback_output / "lidar.lvx2") &&
              ReadText(rollback_result) == "competitor result\n" &&
              fs::is_regular_file(rollback_output.string() + ".displaced-for-test/lidar.lvx2") &&
              ReadText(recording) == source_before,
          "export rollback deleted a competitor that replaced the published output directory");

  const fs::path overflow_recording = directory / "udp-counter-overflow.mcap";
  {
    McapSessionWriter writer(overflow_recording, descriptor_bytes, identity);
    // 每 scan 的两个半帧各产生一包；32,769 scans 用少量点跨过 16-bit package 边界。
    for (std::uint32_t sequence = 0; sequence < 32'769; ++sequence) {
      const std::uint64_t timebase_ns = 2'000'000'000ULL + sequence * 100'000'000ULL;
      const auto payload = MakeLidarPayload(
          sequence, timebase_ns, 42,
          {{0, 0.0F, 0.0F, 0.0F, 0, 0}, {50'000'000, 0.0F, 0.0F, 0.0F, 0, 0}},
          identity.descriptor_sha256);
      writer.Write("/sim/lidar/points", sequence, timebase_ns, timebase_ns, payload);
    }
    writer.Finalize();
  }
  const fs::path overflow_output = directory / "udp-counter-overflow-export";
  const fs::path overflow_result = directory / "udp-counter-overflow.json";
  const ExportRun overflow_run = RunExport(
      root, overflow_recording, overflow_output, overflow_result, 1'048'576);
  Require(overflow_run.exit_code != 0 &&
              overflow_run.error.find("LVX2 UDP counter overflows uint16") != std::string::npos &&
              !fs::exists(overflow_output) && !fs::exists(overflow_result),
          "export did not fail closed before the LVX2 UDP counter wrapped");

  const fs::path exact_counter_recording = directory / "udp-counter-exact-limit.mcap";
  {
    McapSessionWriter writer(exact_counter_recording, descriptor_bytes, identity);
    // 32,768 scans * 2 half-frame packages = exactly 65,536 packages. RLIMIT keeps output bounded.
    for (std::uint32_t sequence = 0; sequence < 32'768; ++sequence) {
      const std::uint64_t timebase_ns = 20'000'000'000ULL + sequence * 100'000'000ULL;
      const auto payload = MakeLidarPayload(
          sequence, timebase_ns, 42,
          {{0, 0.0F, 0.0F, 0.0F, 0, 0}, {50'000'000, 0.0F, 0.0F, 0.0F, 0, 0}},
          identity.descriptor_sha256);
      writer.Write("/sim/lidar/points", sequence, timebase_ns, timebase_ns, payload);
    }
    writer.Finalize();
  }
  const fs::path exact_counter_output = directory / "udp-counter-exact-limit-export";
  const fs::path exact_counter_result = directory / "udp-counter-exact-limit.json";
  const ExportRun exact_counter_run = RunExport(
      root, exact_counter_recording, exact_counter_output, exact_counter_result, 131'072);
  Require(exact_counter_run.exit_code != 0 &&
              exact_counter_run.error.find("LVX2 UDP counter overflows uint16") == std::string::npos &&
              !fs::exists(exact_counter_output) && !fs::exists(exact_counter_result),
          "export rejected exactly 65,536 LVX2 packages as a UDP counter overflow");

  const fs::path reversed_time_recording = directory / "reversed-absolute-time.mcap";
  {
    McapSessionWriter writer(reversed_time_recording, descriptor_bytes, identity);
    const auto earlier_scan = MakeLidarPayload(
        10, 3'000'000'000, 42, {{99'000'000, 1.0F, 0.0F, 0.0F, 1, 0}},
        identity.descriptor_sha256);
    const auto later_scan = MakeLidarPayload(
        11, 3'000'000'001, 42, {{0, 2.0F, 0.0F, 0.0F, 2, 0}},
        identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 10, 3'000'000'000, 3'000'000'000, earlier_scan);
    writer.Write("/sim/lidar/points", 11, 3'000'000'001, 3'000'000'001, later_scan);
    writer.Finalize();
  }
  const std::string reversed_time_before = ReadText(reversed_time_recording);
  const fs::path reversed_time_output = directory / "reversed-absolute-time-export";
  const fs::path reversed_time_result = directory / "reversed-absolute-time.json";
  const ExportRun reversed_time_run =
      RunExport(root, reversed_time_recording, reversed_time_output, reversed_time_result);
  Require(reversed_time_run.exit_code != 0 && !fs::exists(reversed_time_output) &&
              !fs::exists(reversed_time_result) &&
              reversed_time_run.error.find("LiDAR absolute point timestamps are not ordered across scans") !=
                  std::string::npos &&
              ReadText(reversed_time_recording) == reversed_time_before,
          "export accepted points whose absolute time moved backwards across scans");

  const fs::path metadata_time_recording = directory / "metadata-time-mismatch.mcap";
  {
    McapSessionWriter writer(metadata_time_recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(
        20, 4'000'000'000, 42, {{0, 1.0F, 0.0F, 0.0F, 1, 0}},
        identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 20, 4'000'000'000, 3'999'999'999, payload);
    writer.Finalize();
  }
  const std::string metadata_time_before = ReadText(metadata_time_recording);
  const fs::path metadata_time_output = directory / "metadata-time-mismatch-export";
  const fs::path metadata_time_result = directory / "metadata-time-mismatch.json";
  const ExportRun metadata_time_run =
      RunExport(root, metadata_time_recording, metadata_time_output, metadata_time_result);
  Require(metadata_time_run.exit_code != 0 && !fs::exists(metadata_time_output) &&
              !fs::exists(metadata_time_result) &&
              metadata_time_run.error.find("LiDAR MCAP metadata differs from payload timebase") !=
                  std::string::npos &&
              ReadText(metadata_time_recording) == metadata_time_before,
          "export accepted LiDAR MCAP metadata that differs from payload timebase");

  const fs::path metadata_log_time_recording = directory / "metadata-log-time-mismatch.mcap";
  {
    McapSessionWriter writer(metadata_log_time_recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(
        21, 4'100'000'000, 42, {{0, 1.0F, 0.0F, 0.0F, 1, 0}},
        identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 21, 4'099'999'999, 4'100'000'000, payload);
    writer.Finalize();
  }
  const std::string metadata_log_time_before = ReadText(metadata_log_time_recording);
  const fs::path metadata_log_time_output = directory / "metadata-log-time-mismatch-export";
  const fs::path metadata_log_time_result = directory / "metadata-log-time-mismatch.json";
  const ExportRun metadata_log_time_run =
      RunExport(root, metadata_log_time_recording, metadata_log_time_output, metadata_log_time_result);
  Require(metadata_log_time_run.exit_code != 0 && !fs::exists(metadata_log_time_output) &&
              !fs::exists(metadata_log_time_result) &&
              metadata_log_time_run.error.find("LiDAR MCAP metadata differs from payload timebase") !=
                  std::string::npos &&
              ReadText(metadata_log_time_recording) == metadata_log_time_before,
          "export accepted LiDAR MCAP log time that differs from payload timebase");

  // Coverage-first：冻结的 writer 检查必须拒绝不可编码坐标，且不得触及 MCAP。
  const std::array<float, 4> invalid_coordinates{
      std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
      2'147'484.0F, -2'147'484.0F};
  for (std::size_t index = 0; index < invalid_coordinates.size(); ++index) {
    const fs::path invalid_recording = directory / ("invalid-coordinate-" + std::to_string(index) + ".mcap");
    {
      McapSessionWriter writer(invalid_recording, descriptor_bytes, identity);
      const auto payload = MakeLidarPayload(
          100 + index, 6'000'000'000 + index * 100'000'000, 42,
          {{0, invalid_coordinates[index], 0.0F, 0.0F, 1, 1}}, identity.descriptor_sha256);
      writer.Write("/sim/lidar/points", 100 + index, 6'000'000'000 + index * 100'000'000,
                   6'000'000'000 + index * 100'000'000, payload);
      writer.Finalize();
    }
    RequireRejectedAndPreserved(root, invalid_recording, directory / ("invalid-coordinate-output-" + std::to_string(index)),
                                directory / ("invalid-coordinate-result-" + std::to_string(index) + ".json"),
                                index < 2 ? "LiDAR coordinate is not finite"
                                          : "LiDAR coordinate exceeds LVX2 int32 millimeters",
                                "export accepted nonfinite or overflowing LVX2 coordinate");
  }

  const fs::path invalid_offset_recording = directory / "invalid-offset.mcap";
  {
    McapSessionWriter writer(invalid_offset_recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(
        110, 7'000'000'000, 42,
        {{0, 0.0F, 0.0F, 0.0F, 1, 1}, {100'000'000, 0.0F, 0.0F, 0.0F, 1, 1}},
        identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 110, 7'000'000'000, 7'000'000'000, payload);
    writer.Finalize();
  }
  RequireRejectedAndPreserved(root, invalid_offset_recording, directory / "invalid-offset-output",
                              directory / "invalid-offset-result.json",
                              "LiDAR point offsets are outside one ordered 100 ms scan",
                              "export accepted a point at the exclusive 100 ms scan boundary");

  const fs::path reversed_offset_recording = directory / "reversed-offset.mcap";
  {
    McapSessionWriter writer(reversed_offset_recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(
        111, 7'100'000'000, 42,
        {{10, 0.0F, 0.0F, 0.0F, 1, 1}, {9, 0.0F, 0.0F, 0.0F, 1, 1}}, identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 111, 7'100'000'000, 7'100'000'000, payload);
    writer.Finalize();
  }
  RequireRejectedAndPreserved(root, reversed_offset_recording, directory / "reversed-offset-output",
                              directory / "reversed-offset-result.json",
                              "LiDAR point offsets are outside one ordered 100 ms scan",
                              "export accepted offsets that move backwards within one scan");

  const auto write_two_scan_fixture = [&](const fs::path& recording_path,
                                          std::uint32_t first_sequence,
                                          std::uint32_t second_sequence,
                                          std::uint64_t first_timebase,
                                          std::uint64_t second_timebase,
                                          const std::array<std::byte, 32>& first_descriptor,
                                          const std::array<std::byte, 32>& second_descriptor,
                                          std::uint32_t first_world,
                                          std::uint32_t second_world,
                                          std::string_view first_session,
                                          std::string_view second_session,
                                          std::uint32_t first_lidar_id,
                                          std::uint32_t second_lidar_id,
                                          std::string_view first_frame,
                                          std::string_view second_frame) {
    McapSessionWriter writer(recording_path, descriptor_bytes, identity);
    const auto first_payload = MakeLidarPayload(first_sequence, first_timebase, first_lidar_id,
                                                {{0, 0.0F, 0.0F, 0.0F, 1, 1}}, first_descriptor,
                                                first_frame, first_world, first_session);
    const auto second_payload = MakeLidarPayload(second_sequence, second_timebase, second_lidar_id,
                                                 {{0, 0.0F, 0.0F, 0.0F, 1, 1}}, second_descriptor,
                                                 second_frame, second_world, second_session);
    writer.Write("/sim/lidar/points", first_sequence, first_timebase, first_timebase, first_payload);
    writer.Write("/sim/lidar/points", second_sequence, second_timebase, second_timebase, second_payload);
    writer.Finalize();
  };
  const std::array<std::byte, 32> alternate_descriptor = [] {
    std::array<std::byte, 32> value{};
    value.front() = std::byte{0xff};
    return value;
  }();
  const std::array<std::byte, 16> alternate_session = [] {
    std::array<std::byte, 16> value{};
    value.front() = std::byte{0xff};
    return value;
  }();
  const std::string_view alternate_session_view(
      reinterpret_cast<const char*>(alternate_session.data()), alternate_session.size());
  struct TwoScanCase final {
    const char* name;
    std::uint32_t first_sequence;
    std::uint32_t second_sequence;
    std::uint64_t first_timebase;
    std::uint64_t second_timebase;
    bool descriptor_changes;
    std::uint32_t first_world;
    std::uint32_t second_world;
    bool session_changes;
    std::uint32_t first_lidar_id;
    std::uint32_t second_lidar_id;
    std::string_view first_frame;
    std::string_view second_frame;
    std::string_view expected_error;
  };
  const fs::path sequence_gap_recording = directory / "sequence-gap.mcap";
  write_two_scan_fixture(sequence_gap_recording, 120, 122, 8'000'000'000, 8'100'000'000,
                         identity.descriptor_sha256, identity.descriptor_sha256, 7, 7, kSessionId, kSessionId,
                         42, 42, "lidar_link", "lidar_link");
  Require(RunExport(root, sequence_gap_recording, directory / "sequence-gap-output",
                    directory / "sequence-gap-result.json").exit_code == 0,
          "export rejected a forward LiDAR sequence gap");

  const std::array<TwoScanCase, 6> two_scan_cases{{
      {"sequence-backwards", 122, 121, 8'200'000'000, 8'300'000'000, false, 7, 7, false, 42, 42, "lidar_link", "lidar_link", "LiDAR scan sequence or timebase is not strictly continuous"},
      {"timebase-equal", 123, 124, 8'400'000'000, 8'400'000'000, false, 7, 7, false, 42, 42, "lidar_link", "lidar_link", "LiDAR scan sequence or timebase is not strictly continuous"},
      {"timebase-backwards", 125, 126, 8'600'000'000, 8'500'000'000, false, 7, 7, false, 42, 42, "lidar_link", "lidar_link", "LiDAR scan sequence or timebase is not strictly continuous"},
      {"descriptor-change", 127, 128, 8'700'000'000, 8'800'000'000, true, 7, 7, false, 42, 42, "lidar_link", "lidar_link", "MCAP session contains an invalid v2 payload"},
      {"lidar-id-change", 129, 130, 8'900'000'000, 9'000'000'000, false, 7, 7, false, 42, 43, "lidar_link", "lidar_link", "LiDAR device identity changed within the MCAP session"},
      {"frame-id-change", 131, 132, 9'100'000'000, 9'200'000'000, false, 7, 7, false, 42, 42, "lidar_link", "other_lidar", "LiDAR frame_id is not lidar_link"},
  }};
  for (const auto& test_case : two_scan_cases) {
    const fs::path case_recording = directory / (std::string(test_case.name) + ".mcap");
    write_two_scan_fixture(
        case_recording, test_case.first_sequence, test_case.second_sequence, test_case.first_timebase,
        test_case.second_timebase, identity.descriptor_sha256,
        test_case.descriptor_changes ? alternate_descriptor : identity.descriptor_sha256,
        test_case.first_world, test_case.second_world, kSessionId,
        test_case.session_changes ? alternate_session_view : kSessionId, test_case.first_lidar_id,
        test_case.second_lidar_id, test_case.first_frame, test_case.second_frame);
    RequireRejectedAndPreserved(root, case_recording, directory / (std::string(test_case.name) + "-output"),
                                directory / (std::string(test_case.name) + "-result.json"),
                                test_case.expected_error,
                                "export accepted invalid LiDAR session continuity or identity");
  }

  const fs::path world_change_recording = directory / "world-change.mcap";
  write_two_scan_fixture(world_change_recording, 133, 134, 9'300'000'000, 9'400'000'000,
                         identity.descriptor_sha256, identity.descriptor_sha256, 7, 8, kSessionId, kSessionId,
                         42, 42, "lidar_link", "lidar_link");
  RequireRejectedAndPreserved(root, world_change_recording, directory / "world-change-output",
                              directory / "world-change-result.json",
                              "LiDAR payload identity differs from MCAP session identity",
                              "export accepted a LiDAR payload whose world generation differs from the session");

  const fs::path session_change_recording = directory / "session-change.mcap";
  write_two_scan_fixture(session_change_recording, 135, 136, 9'500'000'000, 9'600'000'000,
                         identity.descriptor_sha256, identity.descriptor_sha256, 7, 7, kSessionId,
                         alternate_session_view, 42, 42, "lidar_link", "lidar_link");
  RequireRejectedAndPreserved(root, session_change_recording, directory / "session-change-output",
                              directory / "session-change-result.json",
                              "LiDAR payload identity differs from MCAP session identity",
                              "export accepted a LiDAR payload whose session identity differs from MCAP");

  const fs::path no_lidar_recording = directory / "no-lidar.mcap";
  {
    McapSessionWriter writer(no_lidar_recording, descriptor_bytes, identity);
    const auto wheel = ReadFixture("tests/fixtures/stage4/v2/WheelState.bin");
    writer.Write("/sim/wheel/state", 1, 9'700'000'000, 9'700'000'000, wheel);
    writer.Finalize();
  }
  RequireRejectedAndPreserved(root, no_lidar_recording, directory / "no-lidar-output",
                              directory / "no-lidar-result.json", "MCAP session contains no LiDAR scans",
                              "export accepted an MCAP without LiDAR scans");

  const fs::path zero_message_recording = directory / "zero-message.mcap";
  {
    McapSessionWriter writer(zero_message_recording, descriptor_bytes, identity);
    writer.Finalize();
  }
  RequireRejectedAndPreserved(root, zero_message_recording, directory / "zero-message-output",
                              directory / "zero-message-result.json", "MCAP session contains no LiDAR scans",
                              "export accepted a completed MCAP with zero messages");

  const fs::path empty_scan_recording = directory / "empty-scan.mcap";
  {
    McapSessionWriter writer(empty_scan_recording, descriptor_bytes, identity);
    const auto payload = MakeLidarPayload(140, 9'800'000'000, 42, {}, identity.descriptor_sha256);
    writer.Write("/sim/lidar/points", 140, 9'800'000'000, 9'800'000'000, payload);
    writer.Finalize();
  }
  const fs::path empty_scan_output = directory / "empty-scan-output";
  const fs::path empty_scan_result = directory / "empty-scan-result.json";
  Require(RunExport(root, empty_scan_recording, empty_scan_output, empty_scan_result).exit_code == 0,
          "export rejected a zero-point LiDAR scan");
  const std::string empty_lvx2 = ReadText(empty_scan_output / "lidar.lvx2");
  Require(empty_lvx2.size() == 140 && ReadU64(empty_lvx2, 100) == 116 && ReadU64(empty_lvx2, 124) == 140,
          "zero-point LiDAR scan did not produce exactly two empty 50 ms frames");

  const fs::path occupied_output = directory / "occupied-output";
  const fs::path output_only_result = directory / "output-only-result.json";
  fs::create_directory(occupied_output);
  { std::ofstream marker(occupied_output / "keep.txt"); marker << "keep output\n"; }
  const ExportRun occupied_output_run = RunExport(root, recording, occupied_output, output_only_result);
  Require(occupied_output_run.exit_code != 0 &&
              occupied_output_run.error.find("output-dir must be a new directory") != std::string::npos &&
              ReadText(occupied_output / "keep.txt") == "keep output\n" && !fs::exists(output_only_result) &&
              ReadText(recording) == source_before,
          "export overwrote pre-existing output content");

  const fs::path result_only_output = directory / "result-only-output";
  const fs::path occupied_result = directory / "occupied-result.json";
  { std::ofstream marker(occupied_result); marker << "keep result\n"; }
  const ExportRun occupied_result_run = RunExport(root, recording, result_only_output, occupied_result);
  Require(occupied_result_run.exit_code != 0 &&
              occupied_result_run.error.find("result must be a new file") != std::string::npos &&
              ReadText(occupied_result) == "keep result\n" && !fs::exists(result_only_output) &&
              ReadText(recording) == source_before,
          "export overwrote pre-existing result content");

  fs::remove_all(directory);
  return 0;
}
