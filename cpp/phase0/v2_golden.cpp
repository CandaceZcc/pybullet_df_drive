// 阶段四 Phase-0：以冻结的 C++ Protobuf ABI 解码跨语言 v2 golden bytes。
#include <array>
#include <cerrno>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <fcntl.h>
#include <unistd.h>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/util/json_util.h>

#include "sha256.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

namespace fs = std::filesystem;

int PrintVersion() {
  std::cout << "cxx=17\n"
            << "compiler=gcc-13\n"
            << "ecal=6.1.1\n"
            << "protobuf=33.6\n"
            << "glibcxx_cxx11_abi=1\n";
  return 0;
}

std::string ReadInputFile(const char* raw_path) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::runtime_error("input must be an absolute normalized regular file");
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("input is unreadable");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::string HexDigest(const std::array<std::byte, 32>& digest) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string output;
  output.reserve(digest.size() * 2);
  for (const std::byte value : digest) {
    const unsigned char byte = static_cast<unsigned char>(value);
    output.push_back(kHex[byte >> 4]);
    output.push_back(kHex[byte & 0x0f]);
  }
  return output;
}

template <typename Message>
int DecodeMessage(
    const char* descriptor_path,
    const char* message_name,
    const char* payload_path) {
  const std::string descriptor_bytes = ReadInputFile(descriptor_path);
  google::protobuf::FileDescriptorSet descriptor;
  if (!descriptor.ParseFromString(descriptor_bytes)) {
    throw std::runtime_error("descriptor set is invalid");
  }
  const auto descriptor_sha256 = stage4::Sha256(descriptor_bytes);
  const std::string payload = ReadInputFile(payload_path);
  Message message;
  if (!message.ParseFromString(payload)) {
    throw std::runtime_error(std::string(message_name) + " payload is invalid");
  }
  if (message.descriptor_sha256() != stage4::Bytes(descriptor_sha256)) {
    throw std::runtime_error("payload descriptor digest differs from descriptor set");
  }

  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string message_json;
  const auto status = google::protobuf::util::MessageToJsonString(message, &message_json, options);
  if (!status.ok()) {
    throw std::runtime_error(std::string(message_name) + " JSON conversion failed");
  }
  std::cout << "{\"descriptor_sha256\":\"" << HexDigest(descriptor_sha256)
            << "\",\"message\":" << message_json
            << ",\"message_name\":\"" << message_name << "\",\"payload_sha256\":\""
            << HexDigest(stage4::Sha256(payload)) << "\"}\n";
  return 0;
}

int DecodeTopLevelMessage(
    const char* descriptor_path,
    const char* message_name,
    const char* payload_path) {
  const std::string_view name(message_name);
  if (name == "WheelCommand") {
    return DecodeMessage<slope_sim::interfaces::v2::WheelCommand>(
        descriptor_path, message_name, payload_path);
  }
  if (name == "WheelState") {
    return DecodeMessage<slope_sim::interfaces::v2::WheelState>(
        descriptor_path, message_name, payload_path);
  }
  if (name == "LidarPointCloud") {
    return DecodeMessage<slope_sim::interfaces::v2::LidarPointCloud>(
        descriptor_path, message_name, payload_path);
  }
  if (name == "RtkState") {
    return DecodeMessage<slope_sim::interfaces::v2::RtkState>(
        descriptor_path, message_name, payload_path);
  }
  if (name == "ImuAttitude") {
    return DecodeMessage<slope_sim::interfaces::v2::ImuAttitude>(
        descriptor_path, message_name, payload_path);
  }
  throw std::runtime_error("unsupported v2 message name");
}

template <typename Message>
std::string SerializeDeterministic(const Message& message) {
  std::string bytes(message.ByteSizeLong(), '\0');
  google::protobuf::io::ArrayOutputStream array(
      bytes.data(), static_cast<int>(bytes.size()));
  google::protobuf::io::CodedOutputStream stream(&array);
  stream.SetSerializationDeterministic(true);
  if (!message.SerializeToCodedStream(&stream) || stream.HadError()) {
    throw std::runtime_error("deterministic protobuf serialization failed");
  }
  bytes.resize(stream.ByteCount());
  return bytes;
}

void WriteNewFile(const fs::path& path, const std::string& payload) {
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (descriptor < 0) {
    throw std::runtime_error("fixture output cannot be created exclusively");
  }
  std::size_t written = 0;
  while (written < payload.size()) {
    const auto count = ::write(
        descriptor, payload.data() + written, payload.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      (void)::close(descriptor);
      throw std::runtime_error("fixture output write failed");
    }
    written += static_cast<std::size_t>(count);
  }
  if (::fsync(descriptor) != 0) {
    (void)::close(descriptor);
    throw std::runtime_error("fixture output sync failed");
  }
  if (::close(descriptor) != 0) {
    throw std::runtime_error("fixture output close failed");
  }
}

template <typename Message>
void WriteFixture(
    const fs::path& directory,
    const char* name,
    const Message& message,
    std::string* manifest_entries) {
  const std::string payload = SerializeDeterministic(message);
  WriteNewFile(directory / (std::string(name) + ".bin"), payload);
  if (!manifest_entries->empty()) {
    *manifest_entries += ',';
  }
  *manifest_entries += "\"" + std::string(name) + "\":\"" +
      HexDigest(stage4::Sha256(payload)) + "\"";
}

void SetIdentity(
    std::string* simulation_session_id,
    std::string* descriptor_sha256,
    const std::array<std::byte, 32>& digest) {
  // bytes 字段允许 NUL，必须显式保留完整 16-byte 会话标识。
  simulation_session_id->assign(
      "\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb\xcc\xdd\xee\xff", 16);
  *descriptor_sha256 = stage4::Bytes(digest);
}

int EncodeFixtures(const char* descriptor_path, const char* output_path) {
  const std::string descriptor_bytes = ReadInputFile(descriptor_path);
  google::protobuf::FileDescriptorSet descriptor;
  if (!descriptor.ParseFromString(descriptor_bytes)) {
    throw std::runtime_error("descriptor set is invalid");
  }
  const fs::path output_directory(output_path);
  if (!output_directory.is_absolute() || output_directory.lexically_normal() != output_directory ||
      !fs::is_directory(output_directory) || !fs::is_empty(output_directory)) {
    throw std::runtime_error("output directory must be an empty absolute normalized directory");
  }
  const auto descriptor_sha256 = stage4::Sha256(descriptor_bytes);
  std::string manifest_entries;

  slope_sim::interfaces::v2::WheelCommand command;
  SetIdentity(command.mutable_simulation_session_id(), command.mutable_descriptor_sha256(), descriptor_sha256);
  command.set_timestamp_ns(1000000000);
  command.add_drive_wheel_speed_rad_s(1.5F);
  command.add_drive_wheel_speed_rad_s(-2.25F);
  command.set_sequence(3);
  command.set_world_generation(7);
  command.set_command_generation(11);
  command.set_source_id("golden.command");
  command.set_source_session_id(std::string(
      "\xff\xee\xdd\xcc\xbb\xaa\x99\x88\x77\x66\x55\x44\x33\x22\x11\x00", 16));
  command.set_robot_model("df_mid");
  WriteFixture(output_directory, "WheelCommand", command, &manifest_entries);

  slope_sim::interfaces::v2::WheelState state;
  SetIdentity(state.mutable_simulation_session_id(), state.mutable_descriptor_sha256(), descriptor_sha256);
  state.set_timestamp_ns(1000000000);
  state.add_drive_wheel_speed_rad_s(1.5F);
  state.add_drive_wheel_speed_rad_s(-2.25F);
  state.add_drive_wheel_speed_rad_s(3.75F);
  state.add_drive_wheel_speed_rad_s(-4.5F);
  state.add_steering_wheel_angle_rad(0.25F);
  state.add_steering_wheel_angle_rad(-0.5F);
  state.set_sequence(4);
  state.set_world_generation(7);
  state.set_command_generation(11);
  state.set_robot_model("df_mid");
  state.set_command_authority_state(slope_sim::interfaces::v2::ACTIVE);
  state.set_command_owner_source_id("golden.command");
  state.set_command_owner_source_session_id(command.source_session_id());
  state.set_command_peer_count(1);
  WriteFixture(output_directory, "WheelState", state, &manifest_entries);

  slope_sim::interfaces::v2::LidarPointCloud lidar;
  SetIdentity(lidar.mutable_simulation_session_id(), lidar.mutable_descriptor_sha256(), descriptor_sha256);
  lidar.set_timebase_ns(1000000000);
  lidar.set_frame_id("lidar_front");
  lidar.set_lidar_id(1);
  lidar.set_sequence(6);
  lidar.set_world_generation(7);
  auto* first = lidar.add_points();
  first->set_x(1.0F);
  first->set_y(2.0F);
  first->set_z(3.0F);
  first->set_reflectivity(4);
  first->set_tag(5);
  first->set_line(6);
  auto* second = lidar.add_points();
  second->set_offset_time_ns(100);
  second->set_x(-1.0F);
  second->set_y(-2.0F);
  second->set_z(-3.0F);
  second->set_reflectivity(7);
  second->set_tag(8);
  second->set_line(9);
  lidar.set_point_num(static_cast<uint32_t>(lidar.points_size()));
  WriteFixture(output_directory, "LidarPointCloud", lidar, &manifest_entries);

  slope_sim::interfaces::v2::RtkState rtk;
  SetIdentity(rtk.mutable_simulation_session_id(), rtk.mutable_descriptor_sha256(), descriptor_sha256);
  rtk.set_timestamp_ns(1000000000);
  rtk.set_sequence(5);
  rtk.set_world_generation(7);
  rtk.set_frame_id("world");
  rtk.mutable_left()->set_x_m(1.0);
  rtk.mutable_left()->set_y_m(0.5);
  rtk.mutable_left()->set_z_m(0.2);
  rtk.mutable_center()->set_x_m(1.0);
  rtk.mutable_center()->set_y_m(0.0);
  rtk.mutable_center()->set_z_m(0.2);
  rtk.mutable_right()->set_x_m(1.0);
  rtk.mutable_right()->set_y_m(-0.5);
  rtk.mutable_right()->set_z_m(0.2);
  rtk.set_heading_rad(0.25);
  WriteFixture(output_directory, "RtkState", rtk, &manifest_entries);

  slope_sim::interfaces::v2::ImuAttitude imu;
  SetIdentity(imu.mutable_simulation_session_id(), imu.mutable_descriptor_sha256(), descriptor_sha256);
  imu.set_timestamp_ns(1000000000);
  imu.set_roll_rad(0.1);
  imu.set_pitch_rad(0.2);
  imu.set_sequence(7);
  imu.set_world_generation(7);
  imu.set_frame_id("base_link");
  WriteFixture(output_directory, "ImuAttitude", imu, &manifest_entries);

  WriteNewFile(
      output_directory / "manifest.json",
      "{\"descriptor_sha256\":\"" + HexDigest(descriptor_sha256) +
          "\",\"files\":{" + manifest_entries + "}}\n");
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--version") {
    return PrintVersion();
  }
  try {
    if (argc == 6 && std::string_view(argv[1]) == "decode" &&
        std::string_view(argv[2]) == "--descriptor-set") {
      return DecodeTopLevelMessage(argv[3], argv[4], argv[5]);
    }
    if (argc == 6 && std::string_view(argv[1]) == "encode-fixtures" &&
        std::string_view(argv[2]) == "--descriptor-set" &&
        std::string_view(argv[4]) == "--output-dir") {
      return EncodeFixtures(argv[3], argv[5]);
    }
    std::cerr << "error: unsupported v2_golden command\n";
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 66;
  }
}
