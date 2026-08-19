// 阶段四 C2：以同目录临时文件、fsync 和 rename 原子发布 Recorder segment。
#include "slope_sim/client/atomic_segment.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <unistd.h>

namespace slope_sim::client::v2 {
namespace fs = std::filesystem;

AtomicSegment::AtomicSegment(fs::path final_path) : final_path_(std::move(final_path)) {
  if (!final_path_.is_absolute() || final_path_.lexically_normal() != final_path_ ||
      final_path_.filename().empty() || !fs::is_directory(final_path_.parent_path()) ||
      fs::exists(final_path_)) {
    throw std::invalid_argument("segment final path must be a new file below an existing absolute directory");
  }
  temporary_path_ = final_path_.parent_path() /
      (final_path_.filename().string() + ".partial." + std::to_string(::getpid()));
  file_descriptor_ = ::open(temporary_path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (file_descriptor_ < 0) {
    throw std::runtime_error(std::string("segment temporary create failed: ") + std::strerror(errno));
  }
}

AtomicSegment::~AtomicSegment() {
  if (file_descriptor_ >= 0) (void)::close(file_descriptor_);
  if (!finalized_) (void)::unlink(temporary_path_.c_str());
}

void AtomicSegment::Append(const std::vector<std::byte>& bytes) {
  std::size_t written = 0;
  while (written < bytes.size()) {
    const auto count = ::write(file_descriptor_, bytes.data() + written, bytes.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("segment write failed");
    written += static_cast<std::size_t>(count);
  }
}

void AtomicSegment::Finalize() {
  if (finalized_ || ::fsync(file_descriptor_) != 0) {
    throw std::runtime_error("segment sync or close failed");
  }
  const int file_descriptor = file_descriptor_;
  file_descriptor_ = -1;
  if (::close(file_descriptor) != 0) {
    throw std::runtime_error("segment sync or close failed");
  }
  if (::rename(temporary_path_.c_str(), final_path_.c_str()) != 0) {
    throw std::runtime_error("segment atomic rename failed");
  }
  int directory = ::open(final_path_.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  bool directory_sync_failed = directory < 0;
  if (!directory_sync_failed && ::fsync(directory) != 0) {
    directory_sync_failed = true;
  }
  if (directory >= 0) {
    const int directory_descriptor = directory;
    directory = -1;
    if (::close(directory_descriptor) != 0) {
      directory_sync_failed = true;
    }
  }
  if (directory_sync_failed) {
    // rename 后若目录元数据未持久化，撤销最终名，禁止上层把该会话当作已发布。
    (void)::unlink(final_path_.c_str());
    throw std::runtime_error("segment directory sync failed");
  }
  finalized_ = true;
}

}  // namespace slope_sim::client::v2
