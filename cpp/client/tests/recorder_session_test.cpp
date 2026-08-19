// 阶段四 C2：锁定 Recorder callback 仅入队，consumer drain 失败要求安全停车。
#include "slope_sim/client/recorder_session.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <mcap/mcap.hpp>
#include <sys/syscall.h>
#include <unistd.h>

namespace {

std::atomic<bool> block_next_write{false};
std::atomic<bool> write_is_blocked{false};
std::atomic<bool> release_write{false};

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

slope_sim::client::v2::McapSessionIdentity TestIdentity() {
  return {
      {std::byte{0x00}, std::byte{0x11}, std::byte{0x22}, std::byte{0x33},
       std::byte{0x44}, std::byte{0x55}, std::byte{0x66}, std::byte{0x77},
       std::byte{0x88}, std::byte{0x99}, std::byte{0xaa}, std::byte{0xbb},
       std::byte{0xcc}, std::byte{0xdd}, std::byte{0xee}, std::byte{0xff}},
      {std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef},
       std::byte{0x01}, std::byte{0x23}, std::byte{0x45}, std::byte{0x67},
       std::byte{0x89}, std::byte{0xab}, std::byte{0xcd}, std::byte{0xef}},
      7,
      "flat-obstacles-20",
      std::string(slope_sim::client::v2::kMid360PatternVersion),
      slope_sim::client::v2::kMid360PatternSha256,
  };
}

}  // namespace

// 测试专用的 write 截获器：精确把下一次 MCAP 写入暂停，验证 callback 不持有 writer 锁。
extern "C" ssize_t write(int descriptor, const void* buffer, size_t size) {
  if (block_next_write.exchange(false)) {
    write_is_blocked.store(true);
    while (!release_write.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return static_cast<ssize_t>(::syscall(SYS_write, descriptor, buffer, size));
}

int main() {
  namespace fs = std::filesystem;
  using slope_sim::client::v2::RecorderEnqueueResult;
  using slope_sim::client::v2::RecorderSession;
  using slope_sim::client::v2::RecorderSessionState;

  const fs::path directory = fs::temp_directory_path() / ("slope-sim-recorder-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const std::vector<std::byte> descriptor{std::byte{0x0a}, std::byte{0x02}, std::byte{0x76}, std::byte{0x32}};
  const fs::path final_path = directory / "recorded.mcap";
  RecorderSession recorder(2, final_path, descriptor, TestIdentity());
  Require(recorder.Enqueue({"/sim/wheel/state", {std::byte{0x21}, std::byte{0x22}}, 3, 1000, 900}) ==
              RecorderEnqueueResult::kAccepted,
          "RecorderSession rejected an available queue slot");
  Require(recorder.state() == RecorderSessionState::kRecording,
          "RecorderSession unexpectedly requires safe stop");
  Require(!fs::exists(final_path), "RecorderSession published before consumer drain and finalize");
  Require(recorder.DrainOne(), "RecorderSession did not drain the accepted raw frame");
  Require(!recorder.DrainOne(), "RecorderSession drained a non-existent frame");
  Require(recorder.Finalize(), "RecorderSession did not finalize a drained session");

  mcap::McapReader reader;
  Require(reader.open(final_path.string()).ok(), "RecorderSession final MCAP is unreadable");
  std::size_t count = 0;
  for (const auto& view : reader.readMessages()) {
    Require(view.channel->topic == "/sim/wheel/state", "RecorderSession changed the raw topic");
    Require(view.message.sequence == 3 && view.message.logTime == 1000 && view.message.publishTime == 900,
            "RecorderSession changed message timing or sequence");
    Require(view.message.dataSize == 2 && view.message.data[0] == std::byte{0x21} &&
                view.message.data[1] == std::byte{0x22},
            "RecorderSession changed raw payload bytes");
    ++count;
  }
  Require(count == 1, "RecorderSession did not persist exactly one frame");

  const fs::path overflow_path = directory / "overflow.mcap";
  RecorderSession overflow(1, overflow_path, descriptor, TestIdentity());
  Require(overflow.Enqueue({"/sim/wheel/state", {std::byte{0x31}}, 4, 1100, 1000}) ==
              RecorderEnqueueResult::kAccepted,
          "overflow test first frame was rejected");
  Require(overflow.Enqueue({"/sim/imu/attitude", {std::byte{0x32}}, 5, 1200, 1100}) ==
              RecorderEnqueueResult::kOverflow,
          "overflow test did not report a full queue");
  Require(overflow.state() == RecorderSessionState::kSafeStopRequired,
          "queue overflow did not require safe stop");
  Require(overflow.Enqueue({"/sim/imu/attitude", {std::byte{0x33}}, 6, 1300, 1200}) ==
              RecorderEnqueueResult::kFaulted,
          "safe-stop RecorderSession accepted or misclassified a later frame");
  Require(!overflow.DrainOne(), "failed RecorderSession must not continue consuming frames");

  const fs::path concurrent_path = directory / "concurrent.mcap";
  RecorderSession concurrent(2, concurrent_path, descriptor, TestIdentity());
  Require(concurrent.Enqueue({"/sim/wheel/state", {std::byte{0x41}}, 7, 1400, 1300}) ==
              RecorderEnqueueResult::kAccepted,
          "concurrent test first frame was rejected");
  block_next_write.store(true);
  release_write.store(false);
  write_is_blocked.store(false);
  std::atomic<bool> drain_result{false};
  std::thread consumer([&concurrent, &drain_result] { drain_result.store(concurrent.DrainOne()); });
  const auto write_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!write_is_blocked.load() && std::chrono::steady_clock::now() < write_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Require(write_is_blocked.load(), "test did not block the Recorder MCAP write");

  std::atomic<bool> enqueue_returned{false};
  std::atomic<RecorderEnqueueResult> enqueue_result{RecorderEnqueueResult::kFaulted};
  std::thread callback([&concurrent, &enqueue_returned, &enqueue_result] {
    enqueue_result.store(concurrent.Enqueue({"/sim/imu/attitude", {std::byte{0x42}}, 8, 1500, 1400}));
    enqueue_returned.store(true);
  });
  const auto callback_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (!enqueue_returned.load() && std::chrono::steady_clock::now() < callback_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool callback_completed_while_write_blocked = enqueue_returned.load();
  release_write.store(true);
  callback.join();
  consumer.join();
  Require(callback_completed_while_write_blocked,
          "Recorder callback Enqueue waited for the blocked MCAP writer");
  Require(enqueue_result.load() == RecorderEnqueueResult::kAccepted,
          "Recorder callback did not enqueue while writer was blocked");
  Require(drain_result.load(), "Recorder consumer did not finish the blocked write");
  Require(concurrent.DrainOne(), "Recorder did not drain callback frame after writer resumed");
  Require(concurrent.Finalize(), "Recorder did not finalize concurrent session");

  fs::remove_all(directory);
  return 0;
}
