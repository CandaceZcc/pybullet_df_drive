// 阶段四 D：真实 eCAL 到 ROS 2 WheelState Bridge 的最小端到端验收。
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include <ecal/ecal.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "slope_sim_interfaces_v2.pb.h"
#include "slope_sim/client/v2_topics.hpp"

namespace {

namespace fs = std::filesystem;

#ifdef STAGE4_ROS2_BRIDGE_REPLAY_TEST
constexpr const char* kEcalPrefix = "/replay";
constexpr const char* kRosPrefix = "/replay";
constexpr const char* kBridgeMode = "replay";
#else
constexpr const char* kEcalPrefix = "";
constexpr const char* kRosPrefix = "";
constexpr const char* kBridgeMode = "live";
#endif

std::string EcalTopic(const slope_sim::client::v2::TopicContract& contract) {
  return std::string(kEcalPrefix) + contract.topic;
}

std::string RosTopic(const char* suffix) {
  return std::string(kRosPrefix) + suffix;
}

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

std::vector<std::byte> ReadFixture(const std::string& relative_path) {
  const fs::path root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  std::ifstream input(root / relative_path, std::ios::binary);
  Require(static_cast<bool>(input), "fixture cannot be opened");
  const std::string raw{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  return {reinterpret_cast<const std::byte*>(raw.data()),
          reinterpret_cast<const std::byte*>(raw.data()) + raw.size()};
}

eCAL::SDataTypeInformation TopicTypeInfo(
    const slope_sim::client::v2::TopicContract& contract,
    const std::string& descriptor) {
  return {contract.type_name, "proto", descriptor};
}

bool WaitFor(std::function<bool()> condition, int deadline_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

bool NoMessageFor(std::function<void()> spin, const std::atomic<int>& count, int duration_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    spin();
    if (count.load() != 0) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return count.load() == 0;
}

bool PublishUntil(std::function<bool()> publish, std::function<void()> spin,
                  const std::atomic<int>& count, int expected_count, int deadline_ms) {
  return WaitFor([&] {
    if (!publish()) return false;
    spin();
    return count.load() >= expected_count;
  }, deadline_ms);
}

std::optional<int> PublishUntilChildExit(
    pid_t child, std::function<bool()> publish, int deadline_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!publish()) return std::nullopt;
    int status = 0;
    const pid_t observed = ::waitpid(child, &status, WNOHANG);
    if (observed == child) return status;
    if (observed < 0) return std::nullopt;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  int status = 0;
  return ::waitpid(child, &status, WNOHANG) == child ? std::optional<int>(status) : std::nullopt;
}

}  // namespace

int main() {
  const fs::path root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  const fs::path descriptor_path = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc";
  const auto descriptor_bytes = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  const std::string descriptor(reinterpret_cast<const char*>(descriptor_bytes.data()), descriptor_bytes.size());
  const auto payload = ReadFixture("tests/fixtures/stage4/v2/WheelState.bin");
  const auto lidar_payload = ReadFixture("tests/fixtures/stage4/v2/LidarPointCloud.bin");
  const auto rtk_payload = ReadFixture("tests/fixtures/stage4/v2/RtkState.bin");
  const auto imu_payload = ReadFixture("tests/fixtures/stage4/v2/ImuAttitude.bin");
  slope_sim::interfaces::v2::WheelState expected;
  Require(expected.ParseFromArray(payload.data(), static_cast<int>(payload.size())), "WheelState fixture is invalid");
  slope_sim::interfaces::v2::LidarPointCloud expected_lidar;
  Require(expected_lidar.ParseFromArray(lidar_payload.data(), static_cast<int>(lidar_payload.size())),
          "LidarPointCloud fixture is invalid");
  slope_sim::interfaces::v2::RtkState expected_rtk;
  Require(expected_rtk.ParseFromArray(rtk_payload.data(), static_cast<int>(rtk_payload.size())),
          "RtkState fixture is invalid");
  slope_sim::interfaces::v2::ImuAttitude expected_imu;
  Require(expected_imu.ParseFromArray(imu_payload.data(), static_cast<int>(imu_payload.size())),
          "ImuAttitude fixture is invalid");

  Require(eCAL::Initialize("slope-sim-stage4-ros2-bridge-integration"), "eCAL initialization failed");
  rclcpp::init(0, nullptr);
  try {
    const auto& contract = slope_sim::client::v2::TopicContracts()[1];
    eCAL::CPublisher publisher(EcalTopic(contract), TopicTypeInfo(contract, descriptor));
    const auto& lidar_contract = slope_sim::client::v2::TopicContracts()[2];
    eCAL::CPublisher lidar_publisher(EcalTopic(lidar_contract), TopicTypeInfo(lidar_contract, descriptor));
    const auto& rtk_contract = slope_sim::client::v2::TopicContracts()[3];
    eCAL::CPublisher rtk_publisher(EcalTopic(rtk_contract), TopicTypeInfo(rtk_contract, descriptor));
    const auto& imu_contract = slope_sim::client::v2::TopicContracts()[4];
    eCAL::CPublisher imu_publisher(EcalTopic(imu_contract), TopicTypeInfo(imu_contract, descriptor));
    auto node = std::make_shared<rclcpp::Node>("slope_sim_stage4_ros2_bridge_integration");
    std::mutex received_mutex;
    std::optional<sensor_msgs::msg::JointState> received;
    std::atomic<int> received_count{0};
    std::mutex pointcloud_mutex;
    std::optional<sensor_msgs::msg::PointCloud2> pointcloud;
    std::atomic<int> pointcloud_count{0};
    std::mutex livox_mutex;
    std::optional<livox_ros_driver2::msg::CustomMsg> livox;
    std::atomic<int> livox_count{0};
    std::mutex rtk_mutex;
    std::optional<geometry_msgs::msg::PoseStamped> rtk;
    std::atomic<int> rtk_count{0};
    std::mutex imu_mutex;
    std::optional<sensor_msgs::msg::Imu> imu;
    std::atomic<int> imu_count{0};
    std::mutex tf_mutex;
    std::optional<tf2_msgs::msg::TFMessage> transforms;
    std::atomic<int> tf_count{0};
    std::mutex clock_mutex;
    std::optional<rosgraph_msgs::msg::Clock> clock;
    std::atomic<int> clock_count{0};
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        RosTopic("/slope_sim/wheel/state"), 1,
        [&received, &received_mutex, &received_count](sensor_msgs::msg::JointState::SharedPtr message) {
          std::lock_guard<std::mutex> lock(received_mutex);
          received = std::move(*message);
          ++received_count;
        });
    auto pointcloud_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        RosTopic("/slope_sim/lidar/points"), 1,
        [&pointcloud, &pointcloud_mutex, &pointcloud_count](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          std::lock_guard<std::mutex> lock(pointcloud_mutex);
          pointcloud = std::move(*message);
          ++pointcloud_count;
        });
    auto livox_subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        RosTopic("/slope_sim/lidar/custom"), 1,
        [&livox, &livox_mutex, &livox_count](livox_ros_driver2::msg::CustomMsg::SharedPtr message) {
          std::lock_guard<std::mutex> lock(livox_mutex);
          livox = std::move(*message);
          ++livox_count;
        });
    auto rtk_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        RosTopic("/slope_sim/rtk/state"), 1,
        [&rtk, &rtk_mutex, &rtk_count](geometry_msgs::msg::PoseStamped::SharedPtr message) {
          std::lock_guard<std::mutex> lock(rtk_mutex);
          rtk = std::move(*message);
          ++rtk_count;
        });
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::Imu>(
        RosTopic("/slope_sim/imu/attitude"), 1,
        [&imu, &imu_mutex, &imu_count](sensor_msgs::msg::Imu::SharedPtr message) {
          std::lock_guard<std::mutex> lock(imu_mutex);
          imu = std::move(*message);
          ++imu_count;
        });
    auto tf_subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
        RosTopic("/tf"), 1,
        [&transforms, &tf_mutex, &tf_count](tf2_msgs::msg::TFMessage::SharedPtr message) {
          std::lock_guard<std::mutex> lock(tf_mutex);
          transforms = std::move(*message);
          ++tf_count;
        });
    auto clock_subscription = node->create_subscription<rosgraph_msgs::msg::Clock>(
        RosTopic("/clock"), 1,
        [&clock, &clock_mutex, &clock_count](rosgraph_msgs::msg::Clock::SharedPtr message) {
          std::lock_guard<std::mutex> lock(clock_mutex);
          clock = std::move(*message);
          ++clock_count;
        });

    const auto launch_bridge = [&] {
      const pid_t child = ::fork();
      Require(child >= 0, "fork failed");
      if (child != 0) return child;
      const std::string executable = STAGE4_ROS2_BRIDGE_EXECUTABLE;
      const std::string descriptor_arg = descriptor_path.string();
      char* const args[] = {
          const_cast<char*>(executable.c_str()), const_cast<char*>("--descriptor-set"),
          const_cast<char*>(descriptor_arg.c_str()), const_cast<char*>("--duration-ms"),
          const_cast<char*>("5000"), const_cast<char*>("--deadline-ms"),
          const_cast<char*>("5000"), const_cast<char*>("--mode"),
          const_cast<char*>(kBridgeMode), nullptr};
      ::execv(args[0], args);
      _exit(127);
    };
    const pid_t child = launch_bridge();

    Require(WaitFor([&publisher] { return publisher.GetSubscriberCount() == 1; }, 5000),
            "Bridge did not subscribe to WheelState");
    Require(WaitFor([&lidar_publisher] { return lidar_publisher.GetSubscriberCount() == 1; }, 5000),
            "Bridge did not subscribe to LiDAR");
    Require(WaitFor([&rtk_publisher] { return rtk_publisher.GetSubscriberCount() == 1; }, 5000),
            "Bridge did not subscribe to RTK");
    Require(WaitFor([&imu_publisher] { return imu_publisher.GetSubscriberCount() == 1; }, 5000),
            "Bridge did not subscribe to IMU");
    Require(lidar_publisher.Send(lidar_payload.data(), lidar_payload.size()), "LiDAR eCAL publish failed");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, pointcloud_count, 250),
            "Bridge published PointCloud2 before the sensor snapshot was complete");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, livox_count, 250),
            "Bridge published Livox CustomMsg before the sensor snapshot was complete");
    Require(rtk_publisher.Send(rtk_payload.data(), rtk_payload.size()), "RTK eCAL publish failed");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, pointcloud_count, 250),
            "Bridge published PointCloud2 before IMU completed the sensor snapshot");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, livox_count, 250),
            "Bridge published Livox CustomMsg before IMU completed the sensor snapshot");
    Require(imu_publisher.Send(imu_payload.data(), imu_payload.size()), "IMU eCAL publish failed");
    Require(PublishUntil([&] {
      return lidar_publisher.Send(lidar_payload.data(), lidar_payload.size()) &&
          rtk_publisher.Send(rtk_payload.data(), rtk_payload.size()) &&
          imu_publisher.Send(imu_payload.data(), imu_payload.size());
    }, [&node] { rclcpp::spin_some(node); }, pointcloud_count, 1, 5000),
            "Bridge did not publish paired ROS PointCloud2 frame");
    Require(WaitFor([&node, &livox_count] {
      rclcpp::spin_some(node);
      return livox_count.load() == 1;
    }, 5000), "Bridge did not publish paired Livox CustomMsg frame");
    Require(WaitFor([&node, &rtk_count, &imu_count, &tf_count, &clock_count] {
      rclcpp::spin_some(node);
      return rtk_count.load() == 1 && imu_count.load() == 1 && tf_count.load() == 1 && clock_count.load() == 1;
    }, 5000), "Bridge did not publish paired ROS RTK, IMU, TF, and clock frames");
    Require(PublishUntil([&] { return publisher.Send(payload.data(), payload.size()); },
                         [&node] { rclcpp::spin_some(node); }, received_count, 1, 5000),
            "Bridge did not publish first ROS JointState frame");
    Require(publisher.Send(payload.data(), payload.size()), "second WheelState eCAL publish failed");
    Require(WaitFor([&node, &received_count] {
      rclcpp::spin_some(node);
      return received_count.load() == 2;
    }, 5000), "Bridge did not publish both ROS JointState frames");

    int status = 0;
    Require(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "Bridge did not exit cleanly");
    std::lock_guard<std::mutex> lock(received_mutex);
    Require(received->header.stamp.sec == static_cast<std::int32_t>(expected.timestamp_ns() / 1'000'000'000ULL),
            "ROS timestamp seconds changed");
    Require(received->header.stamp.nanosec == expected.timestamp_ns() % 1'000'000'000ULL,
            "ROS timestamp nanoseconds changed");
    Require(received->velocity.size() == static_cast<std::size_t>(expected.drive_wheel_speed_rad_s_size()),
            "ROS drive speed count changed");
    Require(received->position.size() == static_cast<std::size_t>(expected.steering_wheel_angle_rad_size()),
            "ROS steering angle count changed");
    for (int index = 0; index < expected.drive_wheel_speed_rad_s_size(); ++index) {
      Require(received->velocity[static_cast<std::size_t>(index)] == expected.drive_wheel_speed_rad_s(index),
              "ROS drive speed changed");
    }
    for (int index = 0; index < expected.steering_wheel_angle_rad_size(); ++index) {
      Require(received->position[static_cast<std::size_t>(index)] == expected.steering_wheel_angle_rad(index),
              "ROS steering angle changed");
    }
    std::lock_guard<std::mutex> pointcloud_lock(pointcloud_mutex);
    Require(pointcloud->header.frame_id == expected_lidar.frame_id(), "ROS LiDAR frame id changed");
    Require(pointcloud->header.stamp.sec ==
                static_cast<std::int32_t>(expected_lidar.timebase_ns() / 1'000'000'000ULL),
            "ROS LiDAR timestamp seconds changed");
    Require(pointcloud->header.stamp.nanosec == expected_lidar.timebase_ns() % 1'000'000'000ULL,
            "ROS LiDAR timestamp nanoseconds changed");
    Require(pointcloud->height == 1 && pointcloud->width == expected_lidar.point_num(),
            "ROS LiDAR point count changed");
    Require(pointcloud->point_step == 24 && pointcloud->row_step == pointcloud->width * 24,
            "ROS LiDAR point layout changed");
    Require(pointcloud->data.size() == pointcloud->row_step, "ROS LiDAR point data size changed");
    Require(pointcloud->fields.size() == 6, "ROS LiDAR field count changed");
    constexpr std::uint8_t kFloat32 = sensor_msgs::msg::PointField::FLOAT32;
    constexpr std::uint8_t kUint32 = sensor_msgs::msg::PointField::UINT32;
    const auto& fields = pointcloud->fields;
    Require(fields[0].name == "x" && fields[0].offset == 0 && fields[0].datatype == kFloat32,
            "ROS LiDAR x field changed");
    Require(fields[1].name == "y" && fields[1].offset == 4 && fields[1].datatype == kFloat32,
            "ROS LiDAR y field changed");
    Require(fields[2].name == "z" && fields[2].offset == 8 && fields[2].datatype == kFloat32,
            "ROS LiDAR z field changed");
    Require(fields[3].name == "reflectivity" && fields[3].offset == 12 && fields[3].datatype == kUint32,
            "ROS LiDAR reflectivity field changed");
    Require(fields[4].name == "tag" && fields[4].offset == 16 && fields[4].datatype == kUint32,
            "ROS LiDAR tag field changed");
    Require(fields[5].name == "line" && fields[5].offset == 20 && fields[5].datatype == kUint32,
            "ROS LiDAR line field changed");
    const auto& expected_point = expected_lidar.points(0);
    float first_x = 0.0F;
    float first_y = 0.0F;
    float first_z = 0.0F;
    std::uint32_t first_reflectivity = 0;
    std::memcpy(&first_x, pointcloud->data.data(), sizeof(first_x));
    std::memcpy(&first_y, pointcloud->data.data() + 4, sizeof(first_y));
    std::memcpy(&first_z, pointcloud->data.data() + 8, sizeof(first_z));
    std::memcpy(&first_reflectivity, pointcloud->data.data() + 12, sizeof(first_reflectivity));
    Require(first_x == expected_point.x() && first_y == expected_point.y() && first_z == expected_point.z(),
            "ROS LiDAR coordinates changed");
    Require(first_reflectivity == expected_point.reflectivity(), "ROS LiDAR reflectivity changed");
    std::lock_guard<std::mutex> livox_lock(livox_mutex);
    Require(livox->header.frame_id == expected_lidar.frame_id(), "Livox frame id changed");
    Require(livox->timebase == expected_lidar.timebase_ns(), "Livox timebase changed");
    Require(livox->point_num == expected_lidar.point_num() && livox->points.size() == expected_lidar.point_num(),
            "Livox point count changed");
    Require(livox->points.front().x == expected_point.x() && livox->points.front().y == expected_point.y() &&
                livox->points.front().z == expected_point.z(),
            "Livox point coordinates changed");
    Require(livox->points.front().reflectivity == expected_point.reflectivity() &&
                livox->points.front().tag == expected_point.tag() && livox->points.front().line == expected_point.line(),
            "Livox point attributes changed");
    std::lock_guard<std::mutex> rtk_lock(rtk_mutex);
    Require(rtk->header.frame_id == expected_rtk.frame_id(), "ROS RTK frame id changed");
    Require(rtk->header.stamp.sec == static_cast<std::int32_t>(expected_rtk.timestamp_ns() / 1'000'000'000ULL),
            "ROS RTK timestamp seconds changed");
    Require(rtk->header.stamp.nanosec == expected_rtk.timestamp_ns() % 1'000'000'000ULL,
            "ROS RTK timestamp nanoseconds changed");
    Require(rtk->pose.position.x == expected_rtk.center().x_m() &&
                rtk->pose.position.y == expected_rtk.center().y_m() &&
                rtk->pose.position.z == expected_rtk.center().z_m(),
            "ROS RTK center changed");
    Require(rtk->pose.orientation.z == std::sin(expected_rtk.heading_rad() / 2.0) &&
                rtk->pose.orientation.w == std::cos(expected_rtk.heading_rad() / 2.0),
            "ROS RTK heading changed");
    std::lock_guard<std::mutex> imu_lock(imu_mutex);
    Require(imu->header.frame_id == expected_imu.frame_id(), "ROS IMU frame id changed");
    Require(imu->header.stamp.sec == static_cast<std::int32_t>(expected_imu.timestamp_ns() / 1'000'000'000ULL),
            "ROS IMU timestamp seconds changed");
    Require(imu->header.stamp.nanosec == expected_imu.timestamp_ns() % 1'000'000'000ULL,
            "ROS IMU timestamp nanoseconds changed");
    Require(imu->orientation.x == std::sin(expected_imu.roll_rad() / 2.0) * std::cos(expected_imu.pitch_rad() / 2.0) &&
                imu->orientation.y == std::cos(expected_imu.roll_rad() / 2.0) * std::sin(expected_imu.pitch_rad() / 2.0) &&
                imu->orientation.z == -std::sin(expected_imu.roll_rad() / 2.0) * std::sin(expected_imu.pitch_rad() / 2.0) &&
                imu->orientation.w == std::cos(expected_imu.roll_rad() / 2.0) * std::cos(expected_imu.pitch_rad() / 2.0),
            "ROS IMU attitude changed");
    std::lock_guard<std::mutex> tf_lock(tf_mutex);
    Require(transforms->transforms.size() == 2, "ROS TF transform count changed");
    const auto& transform = transforms->transforms.front();
    Require(transform.header.frame_id == "world" && transform.child_frame_id == "base_link",
            "ROS TF frame relation changed");
    Require(transform.header.stamp.sec == static_cast<std::int32_t>(expected_rtk.timestamp_ns() / 1'000'000'000ULL) &&
                transform.header.stamp.nanosec == expected_rtk.timestamp_ns() % 1'000'000'000ULL,
            "ROS TF timestamp changed");
    Require(transform.transform.translation.x == expected_rtk.center().x_m() &&
                transform.transform.translation.y == expected_rtk.center().y_m() &&
                transform.transform.translation.z == expected_rtk.center().z_m(),
            "ROS TF translation changed");
    const double roll_half = expected_imu.roll_rad() / 2.0;
    const double pitch_half = expected_imu.pitch_rad() / 2.0;
    const double yaw_half = expected_rtk.heading_rad() / 2.0;
    Require(transform.transform.rotation.x == std::sin(roll_half) * std::cos(pitch_half) * std::cos(yaw_half) -
                                          std::cos(roll_half) * std::sin(pitch_half) * std::sin(yaw_half) &&
                transform.transform.rotation.y == std::cos(roll_half) * std::sin(pitch_half) * std::cos(yaw_half) +
                                          std::sin(roll_half) * std::cos(pitch_half) * std::sin(yaw_half) &&
                transform.transform.rotation.z == std::cos(roll_half) * std::cos(pitch_half) * std::sin(yaw_half) -
                                          std::sin(roll_half) * std::sin(pitch_half) * std::cos(yaw_half) &&
                transform.transform.rotation.w == std::cos(roll_half) * std::cos(pitch_half) * std::cos(yaw_half) +
                                          std::sin(roll_half) * std::sin(pitch_half) * std::sin(yaw_half),
            "ROS TF rotation changed");
    const auto& lidar_transform = transforms->transforms[1];
    Require(lidar_transform.header.frame_id == "base_link" &&
                lidar_transform.child_frame_id == expected_lidar.frame_id(),
            "ROS LiDAR TF frame relation changed");
    Require(lidar_transform.header.stamp.sec == transform.header.stamp.sec &&
                lidar_transform.header.stamp.nanosec == transform.header.stamp.nanosec,
            "ROS LiDAR TF timestamp changed");
    Require(lidar_transform.transform.translation.x == 0.0 &&
                lidar_transform.transform.translation.y == 0.0 &&
                lidar_transform.transform.translation.z == 0.105 &&
                lidar_transform.transform.rotation.x == 0.0 &&
                lidar_transform.transform.rotation.y == 0.0 &&
                lidar_transform.transform.rotation.z == 0.0 &&
                lidar_transform.transform.rotation.w == 1.0,
            "ROS LiDAR TF mount changed");
    std::lock_guard<std::mutex> clock_lock(clock_mutex);
    Require(clock->clock.sec == static_cast<std::int32_t>(expected_lidar.timebase_ns() / 1'000'000'000ULL) &&
                clock->clock.nanosec == expected_lidar.timebase_ns() % 1'000'000'000ULL,
            "ROS clock changed");

    pointcloud_count.store(0);
    livox_count.store(0);
    rtk_count.store(0);
    imu_count.store(0);
    tf_count.store(0);
    clock_count.store(0);
    const pid_t mismatched_child = launch_bridge();
    Require(WaitFor([&lidar_publisher] { return lidar_publisher.GetSubscriberCount() == 1; }, 5000),
            "Mismatched Bridge did not subscribe to LiDAR");
    Require(WaitFor([&rtk_publisher] { return rtk_publisher.GetSubscriberCount() == 1; }, 5000),
            "Mismatched Bridge did not subscribe to RTK");
    Require(WaitFor([&imu_publisher] { return imu_publisher.GetSubscriberCount() == 1; }, 5000),
            "Mismatched Bridge did not subscribe to IMU");
    slope_sim::interfaces::v2::ImuAttitude mismatched_imu = expected_imu;
    mismatched_imu.set_world_generation(expected_imu.world_generation() + 1);
    std::string mismatched_imu_payload;
    Require(mismatched_imu.SerializeToString(&mismatched_imu_payload), "Mismatched IMU cannot be serialized");
    const auto mismatched_status = PublishUntilChildExit(mismatched_child, [&] {
      return lidar_publisher.Send(lidar_payload.data(), lidar_payload.size()) &&
          rtk_publisher.Send(rtk_payload.data(), rtk_payload.size()) &&
          imu_publisher.Send(mismatched_imu_payload.data(), mismatched_imu_payload.size());
    }, 5000);
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, pointcloud_count, 250),
            "Bridge published PointCloud2 for mismatched sensor identity");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, livox_count, 250),
            "Bridge published Livox CustomMsg for mismatched sensor identity");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, rtk_count, 250),
            "Bridge published RTK for mismatched sensor identity");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, imu_count, 250),
            "Bridge published IMU for mismatched sensor identity");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, tf_count, 250),
            "Bridge published TF for mismatched sensor identity");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, clock_count, 250),
            "Bridge published clock for mismatched sensor identity");
    Require(mismatched_status.has_value() && WIFEXITED(*mismatched_status) &&
                WEXITSTATUS(*mismatched_status) != 0,
            "Bridge accepted mismatched sensor identity");

    pointcloud_count.store(0);
    livox_count.store(0);
    rtk_count.store(0);
    imu_count.store(0);
    tf_count.store(0);
    clock_count.store(0);
    const pid_t wrong_lidar_frame_child = launch_bridge();
    Require(WaitFor([&lidar_publisher] { return lidar_publisher.GetSubscriberCount() == 1; }, 5000),
            "Wrong-frame Bridge did not subscribe to LiDAR");
    Require(WaitFor([&rtk_publisher] { return rtk_publisher.GetSubscriberCount() == 1; }, 5000),
            "Wrong-frame Bridge did not subscribe to RTK");
    Require(WaitFor([&imu_publisher] { return imu_publisher.GetSubscriberCount() == 1; }, 5000),
            "Wrong-frame Bridge did not subscribe to IMU");
    slope_sim::interfaces::v2::LidarPointCloud wrong_lidar_frame = expected_lidar;
    wrong_lidar_frame.set_frame_id("other_lidar");
    std::string wrong_lidar_frame_payload;
    Require(wrong_lidar_frame.SerializeToString(&wrong_lidar_frame_payload),
            "Wrong-frame LiDAR cannot be serialized");
    const auto wrong_lidar_frame_status = PublishUntilChildExit(wrong_lidar_frame_child, [&] {
      return rtk_publisher.Send(rtk_payload.data(), rtk_payload.size()) &&
          imu_publisher.Send(imu_payload.data(), imu_payload.size()) &&
          lidar_publisher.Send(wrong_lidar_frame_payload.data(), wrong_lidar_frame_payload.size());
    }, 5000);
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, pointcloud_count, 250),
            "Bridge published PointCloud2 for a LiDAR frame outside the v2 contract");
    Require(NoMessageFor([&node] { rclcpp::spin_some(node); }, tf_count, 250),
            "Bridge published TF for a LiDAR frame outside the v2 contract");
    Require(wrong_lidar_frame_status.has_value() && WIFEXITED(*wrong_lidar_frame_status) &&
                WEXITSTATUS(*wrong_lidar_frame_status) != 0,
            "Bridge accepted a LiDAR frame outside the v2 contract");
    rclcpp::shutdown();
    eCAL::Finalize();
  } catch (...) {
    if (rclcpp::ok()) rclcpp::shutdown();
    eCAL::Finalize();
    throw;
  }
  return 0;
}
