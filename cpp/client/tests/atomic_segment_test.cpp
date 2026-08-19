// 阶段四 C2：确认 partial segment 在 finalize 前不可见，之后原子成为最终文件。
#include "slope_sim/client/atomic_segment.hpp"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

namespace {

std::atomic<bool> fail_second_sync{false};
std::atomic<int> sync_count{0};
std::atomic<bool> fail_next_close{false};
std::atomic<int> intercepted_close_count{0};

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

}  // namespace

// 测试专用 fsync 截获器：第一次同步文件成功，第二次目录同步确定性失败。
extern "C" int fsync(int descriptor) {
  const int call_number = sync_count.fetch_add(1);
  if (fail_second_sync.load() && call_number == 1) {
    errno = EIO;
    return -1;
  }
  return static_cast<int>(::syscall(SYS_fsync, descriptor));
}

// 测试 close 的 Linux 语义：即使返回错误，FD 也可能已经被内核释放。
extern "C" int close(int descriptor) {
  if (fail_next_close.exchange(false)) {
    ++intercepted_close_count;
    (void)::syscall(SYS_close, descriptor);
    errno = EIO;
    return -1;
  }
  if (intercepted_close_count.load() > 0) {
    ++intercepted_close_count;
  }
  return static_cast<int>(::syscall(SYS_close, descriptor));
}

int main() {
  namespace fs = std::filesystem;
  const fs::path directory = fs::temp_directory_path() / ("slope-sim-segment-" + std::to_string(::getpid()));
  fs::create_directory(directory);
  const fs::path final_path = directory / "segment.mcap";
  const std::vector<std::byte> payload{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  {
    slope_sim::client::v2::AtomicSegment segment(final_path);
    segment.Append(payload);
    Require(!fs::exists(final_path), "AtomicSegment published before Finalize");
    Require(std::distance(fs::directory_iterator(directory), fs::directory_iterator()) == 1,
            "AtomicSegment did not retain exactly one partial file");
    segment.Finalize();
  }
  std::ifstream input(final_path, std::ios::binary);
  const std::vector<char> recorded{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  Require(recorded.size() == payload.size(), "AtomicSegment final payload size differs");
  for (std::size_t index = 0; index < payload.size(); ++index) {
    Require(static_cast<std::byte>(recorded[index]) == payload[index],
            "AtomicSegment final payload bytes differ");
  }
  Require(std::distance(fs::directory_iterator(directory), fs::directory_iterator()) == 1,
          "AtomicSegment left an unexpected file after Finalize");

  const fs::path failed_path = directory / "failed.mcap";
  {
    slope_sim::client::v2::AtomicSegment segment(failed_path);
    segment.Append(payload);
    sync_count.store(0);
    fail_second_sync.store(true);
    bool rejected = false;
    try {
      segment.Finalize();
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    fail_second_sync.store(false);
    Require(rejected, "AtomicSegment accepted a failed directory fsync");
    Require(!fs::exists(failed_path), "AtomicSegment retained final file after directory fsync failure");
    Require(std::distance(fs::directory_iterator(directory), fs::directory_iterator()) == 1,
            "AtomicSegment left a file after directory fsync failure");
  }

  const fs::path close_failure_path = directory / "close-failure.mcap";
  intercepted_close_count.store(0);
  {
    slope_sim::client::v2::AtomicSegment segment(close_failure_path);
    segment.Append(payload);
    fail_next_close.store(true);
    bool rejected = false;
    try {
      segment.Finalize();
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    Require(rejected, "AtomicSegment accepted a failed file close");
  }
  Require(intercepted_close_count.load() == 1,
          "AtomicSegment closed a file descriptor twice after close failure");
  Require(!fs::exists(close_failure_path), "AtomicSegment retained final file after close failure");

  fs::remove_all(directory);
  return 0;
}
