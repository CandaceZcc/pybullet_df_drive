// 阶段四 C1：在当前 euid 的私有 runtime 目录内实现非阻塞单实例锁。
#include "slope_sim/client/command_instance_lock.hpp"

#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace slope_sim::client::v2 {
namespace {

namespace fs = std::filesystem;

fs::path PrivateRuntimeDirectory() {
  const uid_t effective_uid = geteuid();
  const char* const environment_directory = std::getenv("XDG_RUNTIME_DIR");
  const fs::path directory = environment_directory != nullptr && environment_directory[0] != '\0'
      ? fs::path(environment_directory)
      : fs::path("/run/user") / std::to_string(static_cast<unsigned long>(effective_uid));
  struct stat metadata {};
  if (!directory.is_absolute() || directory.lexically_normal() != directory ||
      lstat(directory.c_str(), &metadata) != 0 || !S_ISDIR(metadata.st_mode) ||
      metadata.st_uid != effective_uid || (metadata.st_mode & 0777) != 0700) {
    throw std::runtime_error("command runtime directory must be an euid-owned 0700 directory");
  }
  return directory;
}

fs::path LockPath() {
  return PrivateRuntimeDirectory() / ("slope-sim-stage4-command-" +
      std::to_string(static_cast<unsigned long>(geteuid())) + ".lock");
}

}  // namespace

CommandInstanceLock::CommandInstanceLock() {
  const fs::path lock_path = LockPath();
  file_descriptor_ = open(lock_path.c_str(), O_CREAT | O_CLOEXEC | O_NOFOLLOW | O_RDWR, 0600);
  if (file_descriptor_ < 0) {
    throw std::runtime_error(std::string("command lock open failed: ") + std::strerror(errno));
  }
  struct stat metadata {};
  if (fstat(file_descriptor_, &metadata) != 0 || !S_ISREG(metadata.st_mode) ||
      metadata.st_uid != geteuid() || (metadata.st_mode & 0777) != 0600) {
    (void)close(file_descriptor_);
    file_descriptor_ = -1;
    throw std::runtime_error("command lock must be an euid-owned 0600 regular file");
  }
  if (flock(file_descriptor_, LOCK_EX | LOCK_NB) != 0) {
    const int error = errno;
    (void)close(file_descriptor_);
    file_descriptor_ = -1;
    if (error == EWOULDBLOCK || error == EAGAIN) {
      throw std::runtime_error("another stage4 command process is already running");
    }
    throw std::runtime_error(std::string("command lock acquisition failed: ") + std::strerror(error));
  }
}

CommandInstanceLock::~CommandInstanceLock() {
  if (file_descriptor_ >= 0) {
    (void)flock(file_descriptor_, LOCK_UN);
    (void)close(file_descriptor_);
  }
}

}  // namespace slope_sim::client::v2
