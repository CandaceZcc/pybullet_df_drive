// 阶段四 C1：验证私有 runtime 锁会拒绝第二进程和不安全预置节点。
#include "slope_sim/client/command_instance_lock.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::string LockPath(const char* runtime_directory) {
  return std::string(runtime_directory) + "/slope-sim-stage4-command-" +
      std::to_string(static_cast<unsigned long>(geteuid())) + ".lock";
}

bool RejectsDefaultLock() {
  try {
    slope_sim::client::v2::CommandInstanceLock lock;
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

}  // namespace

int main() {
  char runtime_template[] = "/tmp/slope-sim-stage4-command-runtime-XXXXXX";
  char* const runtime_directory = mkdtemp(runtime_template);
  Require(runtime_directory != nullptr, "mkdtemp failed");
  Require(chmod(runtime_directory, 0700) == 0, "runtime chmod failed");
  Require(setenv("XDG_RUNTIME_DIR", runtime_directory, 1) == 0, "setenv failed");
  const std::string lock_path = LockPath(runtime_directory);

  {
    slope_sim::client::v2::CommandInstanceLock parent_lock;
    const pid_t child = fork();
    Require(child >= 0, "fork failed");
    if (child == 0) {
      try {
        slope_sim::client::v2::CommandInstanceLock child_lock;
        _exit(EXIT_FAILURE);
      } catch (const std::runtime_error&) {
        _exit(EXIT_SUCCESS);
      }
    }
    int status = 0;
    Require(waitpid(child, &status, 0) == child, "waitpid failed");
    Require(WIFEXITED(status), "child did not exit");
    Require(WEXITSTATUS(status) == EXIT_SUCCESS, "child acquired an existing lock");
  }
  struct stat lock_metadata {};
  Require(lstat(lock_path.c_str(), &lock_metadata) == 0, "lock was not created");
  Require(S_ISREG(lock_metadata.st_mode), "lock is not a regular file");
  Require(lock_metadata.st_uid == geteuid(), "lock owner differs from effective uid");
  Require((lock_metadata.st_mode & 0777) == 0600, "lock mode is not 0600");
  Require(unlink(lock_path.c_str()) == 0, "lock unlink failed");

  const std::string target = std::string(runtime_directory) + "/target";
  const int target_fd = open(target.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  Require(target_fd >= 0, "target create failed");
  Require(close(target_fd) == 0, "target close failed");
  Require(symlink(target.c_str(), lock_path.c_str()) == 0, "symlink setup failed");
  Require(RejectsDefaultLock(), "symlinked lock was accepted");
  Require(unlink(lock_path.c_str()) == 0, "symlink cleanup failed");

  const int insecure_fd = open(lock_path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
  Require(insecure_fd >= 0, "insecure lock setup failed");
  Require(close(insecure_fd) == 0, "insecure lock close failed");
  Require(RejectsDefaultLock(), "insecure existing lock was accepted");
  Require(unlink(lock_path.c_str()) == 0, "insecure lock cleanup failed");
  Require(unlink(target.c_str()) == 0, "target cleanup failed");
  Require(rmdir(runtime_directory) == 0, "runtime directory cleanup failed");
  return 0;
}
