// 阶段四 D：ROS TF 与点云只能由同一 v2 snapshot 生成，不能最近邻拼接。
#include "slope_sim/client/ros_sensor_pair.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "../../common/sha256.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

std::string ReadFixture(const std::string& relative_path) {
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
  std::ifstream input(root / relative_path, std::ios::binary);
  Require(static_cast<bool>(input), "fixture cannot be opened");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

}  // namespace

int main() {
  const std::string descriptor = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  const std::string lidar = ReadFixture("tests/fixtures/stage4/v2/LidarPointCloud.bin");
  const std::string rtk = ReadFixture("tests/fixtures/stage4/v2/RtkState.bin");
  const std::string imu = ReadFixture("tests/fixtures/stage4/v2/ImuAttitude.bin");
  const std::string digest = stage4::Bytes(stage4::Sha256(descriptor));
  const auto pair = slope_sim::client::v2::ValidateRosSensorPair(lidar, rtk, imu, digest);
  Require(pair.timestamp_ns == 1'000'000'000 && pair.world_generation == 7,
          "ROS sensor pair changed the shared snapshot identity");

  slope_sim::interfaces::v2::ImuAttitude mismatched;
  Require(mismatched.ParseFromString(imu), "IMU fixture cannot be parsed");
  mismatched.set_timestamp_ns(mismatched.timestamp_ns() + 1);
  bool rejected = false;
  try {
    static_cast<void>(slope_sim::client::v2::ValidateRosSensorPair(
        lidar, rtk, mismatched.SerializeAsString(), digest));
  } catch (const std::exception&) {
    rejected = true;
  }
  Require(rejected, "ROS sensor pair accepted mismatched timestamps");
  return 0;
}
