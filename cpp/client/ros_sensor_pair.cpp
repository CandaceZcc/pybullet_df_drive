// 阶段四 D：ROS 下游适配复用 v2 原始 payload 判据，并拒绝跨快照组合。
#include "slope_sim/client/ros_sensor_pair.hpp"

#include <algorithm>
#include <stdexcept>

#include "slope_sim/client/raw_v2_payload.hpp"
#include "slope_sim_interfaces_v2.pb.h"

namespace slope_sim::client::v2 {

RosSensorPair ValidateRosSensorPair(
    std::string_view lidar_payload,
    std::string_view rtk_payload,
    std::string_view imu_payload,
    std::string_view expected_descriptor_sha256) {
  if (ValidateRawV2Payload("/sim/lidar/points", lidar_payload, expected_descriptor_sha256) !=
          RawV2PayloadValidation::kValid ||
      ValidateRawV2Payload("/sim/rtk/state", rtk_payload, expected_descriptor_sha256) !=
          RawV2PayloadValidation::kValid ||
      ValidateRawV2Payload("/sim/imu/attitude", imu_payload, expected_descriptor_sha256) !=
          RawV2PayloadValidation::kValid) {
    throw std::invalid_argument("ROS sensor pair contains an invalid v2 payload");
  }
  slope_sim::interfaces::v2::LidarPointCloud lidar;
  slope_sim::interfaces::v2::RtkState rtk;
  slope_sim::interfaces::v2::ImuAttitude imu;
  if (!lidar.ParseFromArray(lidar_payload.data(), static_cast<int>(lidar_payload.size())) ||
      !rtk.ParseFromArray(rtk_payload.data(), static_cast<int>(rtk_payload.size())) ||
      !imu.ParseFromArray(imu_payload.data(), static_cast<int>(imu_payload.size()))) {
    throw std::runtime_error("validated ROS sensor payload cannot be parsed");
  }
  if (lidar.simulation_session_id() != rtk.simulation_session_id() ||
      lidar.simulation_session_id() != imu.simulation_session_id() ||
      lidar.world_generation() != rtk.world_generation() ||
      lidar.world_generation() != imu.world_generation() ||
      lidar.timebase_ns() != rtk.timestamp_ns() || lidar.timebase_ns() != imu.timestamp_ns()) {
    throw std::invalid_argument("ROS sensor pair must share session, generation, and timestamp");
  }
  RosSensorPair result{};
  for (std::size_t index = 0; index < result.simulation_session_id.size(); ++index) {
    result.simulation_session_id[index] = static_cast<std::byte>(
        static_cast<unsigned char>(lidar.simulation_session_id()[index]));
  }
  result.world_generation = lidar.world_generation();
  result.timestamp_ns = lidar.timebase_ns();
  return result;
}

}  // namespace slope_sim::client::v2
