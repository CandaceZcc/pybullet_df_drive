// 阶段四 D：真实 eCAL Replay 验收，验证完成 MCAP 的四类输出原样进入隔离 topic。
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include <ecal/ecal.h>

#include "../../common/sha256.hpp"
#include "slope_sim/client/mcap_session_writer.hpp"
#include "slope_sim/client/v2_topics.hpp"

namespace {

namespace fs = std::filesystem;

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

eCAL::SDataTypeInformation TopicTypeInfo(
    const slope_sim::client::v2::TopicContract& contract,
    const std::string& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = contract.type_name;
  info.encoding = "proto";
  info.descriptor = descriptor;
  return info;
}

bool Complete(const std::array<std::vector<std::byte>, 4>& received) {
  for (const auto& payload : received) {
    if (payload.empty()) return false;
  }
  return true;
}

}  // namespace

int main() {
  using slope_sim::client::v2::McapSessionIdentity;
  using slope_sim::client::v2::McapSessionWriter;
  using slope_sim::client::v2::TopicContracts;

  const fs::path directory = fs::temp_directory_path() / ("slope-sim-replay-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const fs::path recording = directory / "outputs.mcap";
  const fs::path descriptor_path = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
      "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc";
  const auto descriptor_bytes = ReadFixture("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc");
  const std::string descriptor(reinterpret_cast<const char*>(descriptor_bytes.data()), descriptor_bytes.size());
  McapSessionIdentity identity{
      {std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
       std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
       std::byte{0x88}, std::byte{0x99}, std::byte{0xaa}, std::byte{0xbb},
       std::byte{0xcc}, std::byte{0xdd}, std::byte{0xee}, std::byte{0xff}},
      stage4::Sha256(descriptor),
      7,
      "replay-integration-scene",
      std::string(slope_sim::client::v2::kMid360PatternVersion),
      slope_sim::client::v2::kMid360PatternSha256,
  };
  const std::array<std::string, 4> fixture_names{
      "WheelState.bin", "LidarPointCloud.bin", "RtkState.bin", "ImuAttitude.bin"};
  std::array<std::vector<std::byte>, 4> expected{};
  {
    McapSessionWriter writer(recording, descriptor_bytes, identity);
    const auto& contracts = TopicContracts();
    for (std::size_t index = 0; index < expected.size(); ++index) {
      expected[index] = ReadFixture("tests/fixtures/stage4/v2/" + fixture_names[index]);
      writer.Write(contracts[index + 1].topic, static_cast<std::uint32_t>(index + 1),
                   1'000 + index, 900 + index, expected[index]);
    }
    writer.Finalize();
  }

  Require(eCAL::Initialize("slope-sim-stage4-replay-integration"), "eCAL initialization failed");
  std::array<std::vector<std::byte>, 4> received{};
  try {
    {
      const auto& contracts = TopicContracts();
      std::array<std::unique_ptr<eCAL::CSubscriber>, 4> subscribers;
      for (std::size_t index = 0; index < subscribers.size(); ++index) {
        subscribers[index] = std::make_unique<eCAL::CSubscriber>(
            std::string("/replay") + contracts[index + 1].topic,
            TopicTypeInfo(contracts[index + 1], descriptor));
        subscribers[index]->SetReceiveCallback(
            [&received, index](const eCAL::STopicId&, const eCAL::SDataTypeInformation&,
                               const eCAL::SReceiveCallbackData& data) {
              if (data.buffer_size > 0 && data.buffer != nullptr) {
                const auto* bytes = static_cast<const std::byte*>(data.buffer);
                received[index] = {bytes, bytes + data.buffer_size};
              }
            });
      }

      const fs::path result = directory / "replay.json";
      const pid_t child = ::fork();
      Require(child >= 0, "fork failed");
      if (child == 0) {
        const std::string executable = STAGE4_REPLAY_EXECUTABLE;
        const std::string input = recording.string();
        const std::string descriptor_arg = descriptor_path.string();
        const std::string deadline = "10000";
        const std::string result_arg = result.string();
        char* const args[] = {
            const_cast<char*>(executable.c_str()), const_cast<char*>("--input"),
            const_cast<char*>(input.c_str()), const_cast<char*>("--descriptor-set"),
            const_cast<char*>(descriptor_arg.c_str()), const_cast<char*>("--deadline-ms"),
            const_cast<char*>(deadline.c_str()), const_cast<char*>("--result"),
            const_cast<char*>(result_arg.c_str()), nullptr};
        ::execv(args[0], args);
        _exit(127);
      }

      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (std::chrono::steady_clock::now() < deadline && !Complete(received)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      int status = 0;
      Require(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
              "replay participant did not exit cleanly");
      for (std::size_t index = 0; index < received.size(); ++index) {
        Require(received[index] == expected[index], "replay changed an output raw payload");
      }
      std::ifstream result_input(result);
      const std::string result_json{std::istreambuf_iterator<char>(result_input), std::istreambuf_iterator<char>()};
      Require(result_json ==
                  "{\"clean_shutdown\":true,\"role\":\"replay\",\"topics\":{"
                  "\"/replay/sim/wheel/state\":1,\"/replay/sim/lidar/points\":1,"
                  "\"/replay/sim/rtk/state\":1,\"/replay/sim/imu/attitude\":1}}\n",
              "replay result is incomplete");
    }
    eCAL::Finalize();
  } catch (...) {
    eCAL::Finalize();
    fs::remove_all(directory);
    throw;
  }
  fs::remove_all(directory);
  return 0;
}
