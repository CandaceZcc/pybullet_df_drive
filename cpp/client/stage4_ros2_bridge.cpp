// 阶段四 D：可选 Jazzy Bridge；下游 ROS 失败不进入 PyBullet/eCAL 核心进程。
#include <charconv>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ecal/ecal.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "../common/sha256.hpp"
#include "slope_sim_interfaces_v2.pb.h"
#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim/client/ros_sensor_pair.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace {
namespace fs = std::filesystem;

std::string ReadFile(const char* raw) {
  const fs::path path(raw);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("descriptor set must be an absolute normalized regular file");
  }
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int Positive(const char* raw, const char* name) {
  int value = 0;
  const auto [end, error] = std::from_chars(raw, raw + std::char_traits<char>::length(raw), value);
  if (error != std::errc{} || *end != '\0' || value < 1 || value > 21600000) {
    throw std::invalid_argument(std::string(name) + " must be in [1, 21600000]");
  }
  return value;
}

/// replay 只允许从 /replay/sim 进入，并把所有 ROS 输出隔离在 /replay 下。
enum class BridgeMode { kLive, kReplay };

BridgeMode ParseMode(const char* raw) {
  const std::string_view mode(raw);
  if (mode == "live") return BridgeMode::kLive;
  if (mode == "replay") return BridgeMode::kReplay;
  throw std::invalid_argument("mode must be live or replay");
}

std::string TopicForMode(BridgeMode mode, std::string_view suffix) {
  return mode == BridgeMode::kReplay ? std::string("/replay") + std::string(suffix) : std::string(suffix);
}

eCAL::SDataTypeInformation TypeInfo(const slope_sim::client::v2::TopicContract& contract,
                                    const std::string& descriptor) {
  return {contract.type_name, "proto", descriptor};
}

int RunBridge(const std::string& descriptor, int duration_ms, int deadline_ms, BridgeMode mode) {
  const auto& contracts = slope_sim::client::v2::TopicContracts();
  const auto& contract = contracts[1];
  const auto& lidar_contract = contracts[2];
  const auto& rtk_contract = contracts[3];
  const auto& imu_contract = contracts[4];
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  if (!eCAL::Initialize("slope-sim-stage4-ros2-bridge")) throw std::runtime_error("eCAL initialization failed");
  try {
    auto node = std::make_shared<rclcpp::Node>(
        mode == BridgeMode::kReplay ? "slope_sim_stage4_ros2_replay_bridge" : "slope_sim_stage4_ros2_bridge");
    auto output = node->create_publisher<sensor_msgs::msg::JointState>(TopicForMode(mode, "/slope_sim/wheel/state"), 1);
    auto pointcloud_output = node->create_publisher<sensor_msgs::msg::PointCloud2>(TopicForMode(mode, "/slope_sim/lidar/points"), 1);
    auto livox_output = node->create_publisher<livox_ros_driver2::msg::CustomMsg>(TopicForMode(mode, "/slope_sim/lidar/custom"), 1);
    auto rtk_output = node->create_publisher<geometry_msgs::msg::PoseStamped>(TopicForMode(mode, "/slope_sim/rtk/state"), 1);
    auto imu_output = node->create_publisher<sensor_msgs::msg::Imu>(TopicForMode(mode, "/slope_sim/imu/attitude"), 1);
    auto clock_output = node->create_publisher<rosgraph_msgs::msg::Clock>(TopicForMode(mode, "/clock"), 1);
    auto tf_output = node->create_publisher<tf2_msgs::msg::TFMessage>(TopicForMode(mode, "/tf"), 1);
    std::atomic<int> accepted{0};
    std::atomic<int> rejected{0};
    std::atomic<int> pending{0};
    struct SensorPayload final {
      std::string raw;
      std::uint64_t timestamp_ns;
    };
    // eCAL 回调可能并发到达；缓存自有字节，禁止组合 callback buffer 的借用内存。
    std::mutex sensor_mutex;
    std::optional<SensorPayload> latest_lidar;
    std::optional<SensorPayload> latest_rtk;
    std::optional<SensorPayload> latest_imu;
    std::optional<slope_sim::client::v2::RosSensorPair> last_sensor_pair;
    enum class MetadataState { kPending, kVerified, kConflict };
    auto metadata_state = [&](const slope_sim::client::v2::TopicContract& expected,
                              const eCAL::SDataTypeInformation& info,
                              const eCAL::SReceiveCallbackData& data) {
      // eCAL can deliver an endpoint before its type metadata is complete.  Such
      // frames remain fail-closed, but must not poison the verified session.
      if (data.buffer_size == 0 || info.name.empty() || info.encoding.empty() || info.descriptor.empty()) {
        return MetadataState::kPending;
      }
      if (data.buffer_size > 0 && data.buffer == nullptr) return MetadataState::kConflict;
      return info.name == expected.type_name && info.encoding == "proto" &&
              stage4::Bytes(stage4::Sha256(info.descriptor)) == digest
          ? MetadataState::kVerified
          : MetadataState::kConflict;
    };
    auto publish_paired_sensors = [&] {
      struct SensorMessages final {
        sensor_msgs::msg::PointCloud2 pointcloud;
        livox_ros_driver2::msg::CustomMsg livox;
        geometry_msgs::msg::PoseStamped rtk;
        sensor_msgs::msg::Imu imu;
        geometry_msgs::msg::TransformStamped world_to_base;
        geometry_msgs::msg::TransformStamped base_to_lidar;
        rosgraph_msgs::msg::Clock clock;
      };
      std::optional<SensorMessages> messages;
      {
        std::lock_guard<std::mutex> lock(sensor_mutex);
        if (!latest_lidar || !latest_rtk || !latest_imu ||
            latest_lidar->timestamp_ns != latest_rtk->timestamp_ns ||
            latest_lidar->timestamp_ns != latest_imu->timestamp_ns) {
          return;
        }
        try {
          const auto pair = slope_sim::client::v2::ValidateRosSensorPair(
              latest_lidar->raw, latest_rtk->raw, latest_imu->raw, digest);
          if (last_sensor_pair && last_sensor_pair->simulation_session_id == pair.simulation_session_id &&
              last_sensor_pair->world_generation == pair.world_generation &&
              last_sensor_pair->timestamp_ns == pair.timestamp_ns) {
            return;
          }
          slope_sim::interfaces::v2::LidarPointCloud lidar;
          slope_sim::interfaces::v2::RtkState rtk;
          slope_sim::interfaces::v2::ImuAttitude imu;
          if (!lidar.ParseFromArray(latest_lidar->raw.data(), static_cast<int>(latest_lidar->raw.size()))) {
            ++rejected;
            return;
          }
          if (!rtk.ParseFromArray(latest_rtk->raw.data(), static_cast<int>(latest_rtk->raw.size())) ||
              !imu.ParseFromArray(latest_imu->raw.data(), static_cast<int>(latest_imu->raw.size()))) {
            ++rejected;
            return;
          }
          sensor_msgs::msg::PointCloud2 cloud;
          cloud.header.stamp.sec = static_cast<std::int32_t>(lidar.timebase_ns() / 1'000'000'000ULL);
          cloud.header.stamp.nanosec = static_cast<std::uint32_t>(lidar.timebase_ns() % 1'000'000'000ULL);
          cloud.header.frame_id = lidar.frame_id();
          cloud.height = 1;
          cloud.width = lidar.point_num();
          cloud.is_bigendian = false;
          cloud.is_dense = true;
          cloud.point_step = 24;
          cloud.row_step = cloud.width * cloud.point_step;
          const auto add_field = [&](const char* name, std::uint32_t offset, std::uint8_t datatype) {
            sensor_msgs::msg::PointField field;
            field.name = name;
            field.offset = offset;
            field.datatype = datatype;
            field.count = 1;
            cloud.fields.push_back(std::move(field));
          };
          add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
          add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
          add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
          add_field("reflectivity", 12, sensor_msgs::msg::PointField::UINT32);
          add_field("tag", 16, sensor_msgs::msg::PointField::UINT32);
          add_field("line", 20, sensor_msgs::msg::PointField::UINT32);
          cloud.data.resize(cloud.row_step);
          livox_ros_driver2::msg::CustomMsg livox;
          livox.header = cloud.header;
          livox.timebase = lidar.timebase_ns();
          livox.point_num = lidar.point_num();
          livox.lidar_id = 0;
          livox.points.reserve(lidar.points_size());
          for (int index = 0; index < lidar.points_size(); ++index) {
            const auto& point = lidar.points(index);
            std::byte* const destination = reinterpret_cast<std::byte*>(cloud.data.data()) + index * cloud.point_step;
            const float x = point.x();
            const float y = point.y();
            const float z = point.z();
            const std::uint32_t reflectivity = point.reflectivity();
            const std::uint32_t tag = point.tag();
            const std::uint32_t line = point.line();
            std::memcpy(destination, &x, sizeof(x));
            std::memcpy(destination + 4, &y, sizeof(y));
            std::memcpy(destination + 8, &z, sizeof(z));
            std::memcpy(destination + 12, &reflectivity, sizeof(reflectivity));
            std::memcpy(destination + 16, &tag, sizeof(tag));
            std::memcpy(destination + 20, &line, sizeof(line));
            livox_ros_driver2::msg::CustomPoint livox_point;
            livox_point.offset_time = 0;
            livox_point.x = x;
            livox_point.y = y;
            livox_point.z = z;
            // v2 保留 32 位字段；Livox 官方 CustomPoint 是 8 位，超范围值必须饱和而非回绕。
            livox_point.reflectivity = static_cast<std::uint8_t>(std::min(reflectivity, 255U));
            livox_point.tag = static_cast<std::uint8_t>(std::min(tag, 255U));
            livox_point.line = static_cast<std::uint8_t>(std::min(line, 255U));
            livox.points.push_back(std::move(livox_point));
          }
          geometry_msgs::msg::PoseStamped rtk_message;
          rtk_message.header.stamp.sec = static_cast<std::int32_t>(rtk.timestamp_ns() / 1'000'000'000ULL);
          rtk_message.header.stamp.nanosec = static_cast<std::uint32_t>(rtk.timestamp_ns() % 1'000'000'000ULL);
          rtk_message.header.frame_id = rtk.frame_id();
          rtk_message.pose.position.x = rtk.center().x_m();
          rtk_message.pose.position.y = rtk.center().y_m();
          rtk_message.pose.position.z = rtk.center().z_m();
          // RTK 的 RIGHT-to-LEFT 航向是 world 平面 yaw，直接编码为 Pose 四元数。
          rtk_message.pose.orientation.z = std::sin(rtk.heading_rad() / 2.0);
          rtk_message.pose.orientation.w = std::cos(rtk.heading_rad() / 2.0);
          sensor_msgs::msg::Imu imu_message;
          imu_message.header.stamp.sec = static_cast<std::int32_t>(imu.timestamp_ns() / 1'000'000'000ULL);
          imu_message.header.stamp.nanosec = static_cast<std::uint32_t>(imu.timestamp_ns() % 1'000'000'000ULL);
          imu_message.header.frame_id = imu.frame_id();
          const double roll_half = imu.roll_rad() / 2.0;
          const double pitch_half = imu.pitch_rad() / 2.0;
          imu_message.orientation.x = std::sin(roll_half) * std::cos(pitch_half);
          imu_message.orientation.y = std::cos(roll_half) * std::sin(pitch_half);
          imu_message.orientation.z = -std::sin(roll_half) * std::sin(pitch_half);
          imu_message.orientation.w = std::cos(roll_half) * std::cos(pitch_half);
          geometry_msgs::msg::TransformStamped world_to_base;
          world_to_base.header = rtk_message.header;
          world_to_base.header.frame_id = "world";
          world_to_base.child_frame_id = "base_link";
          world_to_base.transform.translation.x = rtk_message.pose.position.x;
          world_to_base.transform.translation.y = rtk_message.pose.position.y;
          world_to_base.transform.translation.z = rtk_message.pose.position.z;
          const double yaw_half = rtk.heading_rad() / 2.0;
          // TF 合成 world 的 RTK yaw 与 base_link 的 IMU roll/pitch，保持同一传感器快照。
          world_to_base.transform.rotation.x = std::sin(roll_half) * std::cos(pitch_half) * std::cos(yaw_half) -
              std::cos(roll_half) * std::sin(pitch_half) * std::sin(yaw_half);
          world_to_base.transform.rotation.y = std::cos(roll_half) * std::sin(pitch_half) * std::cos(yaw_half) +
              std::sin(roll_half) * std::cos(pitch_half) * std::sin(yaw_half);
          world_to_base.transform.rotation.z = std::cos(roll_half) * std::cos(pitch_half) * std::sin(yaw_half) -
              std::sin(roll_half) * std::sin(pitch_half) * std::cos(yaw_half);
          world_to_base.transform.rotation.w = std::cos(roll_half) * std::cos(pitch_half) * std::cos(yaw_half) +
              std::sin(roll_half) * std::sin(pitch_half) * std::sin(yaw_half);
          // 四车型的 lidar_link 都是 base_link 上固定 10.5 cm 挂点，独立显示可据此累积点云。
          geometry_msgs::msg::TransformStamped base_to_lidar;
          base_to_lidar.header = world_to_base.header;
          base_to_lidar.header.frame_id = "base_link";
          base_to_lidar.child_frame_id = "lidar_link";
          base_to_lidar.transform.translation.z = 0.105;
          base_to_lidar.transform.rotation.w = 1.0;
          rosgraph_msgs::msg::Clock clock;
          clock.clock.sec = static_cast<std::int32_t>(pair.timestamp_ns / 1'000'000'000ULL);
          clock.clock.nanosec = static_cast<std::uint32_t>(pair.timestamp_ns % 1'000'000'000ULL);
          last_sensor_pair = pair;
          messages = SensorMessages{std::move(cloud), std::move(livox), std::move(rtk_message), std::move(imu_message), std::move(world_to_base), std::move(base_to_lidar), std::move(clock)};
        } catch (const std::exception&) {
          ++rejected;
          return;
        }
      }
      pointcloud_output->publish(std::move(messages->pointcloud));
      livox_output->publish(std::move(messages->livox));
      rtk_output->publish(std::move(messages->rtk));
      imu_output->publish(std::move(messages->imu));
      tf2_msgs::msg::TFMessage transforms;
      transforms.transforms.push_back(std::move(messages->world_to_base));
      transforms.transforms.push_back(std::move(messages->base_to_lidar));
      tf_output->publish(std::move(transforms));
      clock_output->publish(std::move(messages->clock));
    };
    eCAL::CSubscriber input(TopicForMode(mode, contract.topic), TypeInfo(contract, descriptor));
    input.SetReceiveCallback([&](const eCAL::STopicId&, const eCAL::SDataTypeInformation& info,
                                 const eCAL::SReceiveCallbackData& data) {
      const auto metadata = metadata_state(contract, info, data);
      if (metadata != MetadataState::kVerified) {
        if (metadata == MetadataState::kConflict) ++rejected;
        else ++pending;
        return;
      }
      const std::string_view raw(static_cast<const char*>(data.buffer), data.buffer_size);
      if (slope_sim::client::v2::ValidateRawV2Payload(contract.topic, raw, digest) !=
          slope_sim::client::v2::RawV2PayloadValidation::kValid) { ++rejected; return; }
      slope_sim::interfaces::v2::WheelState state;
      if (!state.ParseFromArray(data.buffer, static_cast<int>(data.buffer_size))) { ++rejected; return; }
      sensor_msgs::msg::JointState message;
      message.header.stamp.sec = static_cast<std::int32_t>(state.timestamp_ns() / 1'000'000'000ULL);
      message.header.stamp.nanosec = static_cast<std::uint32_t>(state.timestamp_ns() % 1'000'000'000ULL);
      message.header.frame_id = "base_link";
      message.velocity.assign(state.drive_wheel_speed_rad_s().begin(), state.drive_wheel_speed_rad_s().end());
      message.position.assign(state.steering_wheel_angle_rad().begin(), state.steering_wheel_angle_rad().end());
      output->publish(std::move(message));
      ++accepted;
    });
    eCAL::CSubscriber lidar_input(TopicForMode(mode, lidar_contract.topic), TypeInfo(lidar_contract, descriptor));
    lidar_input.SetReceiveCallback([&](const eCAL::STopicId&, const eCAL::SDataTypeInformation& info,
                                       const eCAL::SReceiveCallbackData& data) {
      const auto metadata = metadata_state(lidar_contract, info, data);
      if (metadata != MetadataState::kVerified) {
        if (metadata == MetadataState::kConflict) ++rejected;
        else ++pending;
        return;
      }
      const std::string raw(static_cast<const char*>(data.buffer), data.buffer_size);
      if (slope_sim::client::v2::ValidateRawV2Payload(lidar_contract.topic, raw, digest) !=
          slope_sim::client::v2::RawV2PayloadValidation::kValid) { ++rejected; return; }
      slope_sim::interfaces::v2::LidarPointCloud lidar;
      if (!lidar.ParseFromString(raw)) { ++rejected; return; }
      if (lidar.frame_id() != "lidar_link") { ++rejected; return; }
      { std::lock_guard<std::mutex> lock(sensor_mutex); latest_lidar = SensorPayload{raw, lidar.timebase_ns()}; }
      publish_paired_sensors();
    });
    eCAL::CSubscriber rtk_input(TopicForMode(mode, rtk_contract.topic), TypeInfo(rtk_contract, descriptor));
    rtk_input.SetReceiveCallback([&](const eCAL::STopicId&, const eCAL::SDataTypeInformation& info,
                                     const eCAL::SReceiveCallbackData& data) {
      const auto metadata = metadata_state(rtk_contract, info, data);
      if (metadata != MetadataState::kVerified) {
        if (metadata == MetadataState::kConflict) ++rejected;
        else ++pending;
        return;
      }
      const std::string raw(static_cast<const char*>(data.buffer), data.buffer_size);
      if (slope_sim::client::v2::ValidateRawV2Payload(rtk_contract.topic, raw, digest) !=
          slope_sim::client::v2::RawV2PayloadValidation::kValid) { ++rejected; return; }
      slope_sim::interfaces::v2::RtkState rtk;
      if (!rtk.ParseFromString(raw)) { ++rejected; return; }
      { std::lock_guard<std::mutex> lock(sensor_mutex); latest_rtk = SensorPayload{raw, rtk.timestamp_ns()}; }
      publish_paired_sensors();
    });
    eCAL::CSubscriber imu_input(TopicForMode(mode, imu_contract.topic), TypeInfo(imu_contract, descriptor));
    imu_input.SetReceiveCallback([&](const eCAL::STopicId&, const eCAL::SDataTypeInformation& info,
                                     const eCAL::SReceiveCallbackData& data) {
      const auto metadata = metadata_state(imu_contract, info, data);
      if (metadata != MetadataState::kVerified) {
        if (metadata == MetadataState::kConflict) ++rejected;
        else ++pending;
        return;
      }
      const std::string raw(static_cast<const char*>(data.buffer), data.buffer_size);
      if (slope_sim::client::v2::ValidateRawV2Payload(imu_contract.topic, raw, digest) !=
          slope_sim::client::v2::RawV2PayloadValidation::kValid) { ++rejected; return; }
      slope_sim::interfaces::v2::ImuAttitude imu;
      if (!imu.ParseFromString(raw)) { ++rejected; return; }
      { std::lock_guard<std::mutex> lock(sensor_mutex); latest_imu = SensorPayload{raw, imu.timestamp_ns()}; }
      publish_paired_sensors();
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
    while (std::chrono::steady_clock::now() < end && std::chrono::steady_clock::now() < deadline && rejected == 0) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (accepted.load() < 1 || rejected.load() != 0) {
      throw std::runtime_error(
          "Bridge did not receive verified WheelState frames: accepted=" +
          std::to_string(accepted.load()) + ", rejected=" +
          std::to_string(rejected.load()) + ", pending=" +
          std::to_string(pending.load()));
    }
    eCAL::Finalize();
    return 0;
  } catch (...) { eCAL::Finalize(); throw; }
}
}  // namespace

int main(int argc, char* argv[]) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--dry-run") {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<rclcpp::Node>("slope_sim_stage4_ros2_bridge");
      if (!rclcpp::ok() || std::string(node->get_name()) != "slope_sim_stage4_ros2_bridge") throw std::runtime_error("ROS Bridge node initialization failed");
      rclcpp::shutdown(); std::cout << "ros2_bridge=ready\n"; return 0;
    }
    if (!((argc == 7 || argc == 9) && std::string(argv[1]) == "--descriptor-set" &&
          std::string(argv[3]) == "--duration-ms" && std::string(argv[5]) == "--deadline-ms" &&
          (argc == 7 || std::string(argv[7]) == "--mode"))) {
      throw std::invalid_argument("ROS Bridge options are incomplete");
    }
    rclcpp::init(argc, argv);
    const int result = RunBridge(
        ReadFile(argv[2]), Positive(argv[4], "duration-ms"), Positive(argv[6], "deadline-ms"),
        argc == 9 ? ParseMode(argv[8]) : BridgeMode::kLive);
    rclcpp::shutdown(); return result;
  } catch (const std::exception& error) { if (rclcpp::ok()) rclcpp::shutdown(); std::cerr << "error: " << error.what() << '\n'; return 64; }
}
