// 阶段四 C1：C++ Command 的跨进程单实例 POSIX 锁。
#pragma once

namespace slope_sim::client::v2 {

/// 进程存活期间独占当前 euid 的私有 runtime 锁；冲突时立即抛出，不等待或删除现有文件。
class CommandInstanceLock final {
 public:
  CommandInstanceLock();
  ~CommandInstanceLock();

  CommandInstanceLock(const CommandInstanceLock&) = delete;
  CommandInstanceLock& operator=(const CommandInstanceLock&) = delete;

 private:
  int file_descriptor_ = -1;
};

}  // namespace slope_sim::client::v2
