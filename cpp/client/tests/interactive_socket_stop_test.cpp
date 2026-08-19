// runSim v2：验证认证 stop 是交互 socket 批次的终止边界。
#define main slope_sim_stage4_command_program_main
#include "../stage4_command.cpp"
#undef main

#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class ScopedDirectory final {
 public:
  ScopedDirectory() {
    std::array<char, 64> pattern{};
    const char* const source = "/tmp/slope-sim-stop-XXXXXX";
    std::strncpy(pattern.data(), source, pattern.size() - 1U);
    if (mkdtemp(pattern.data()) == nullptr || chmod(pattern.data(), 0700) != 0) {
      throw std::runtime_error("temporary socket directory creation failed");
    }
    path_ = pattern.data();
  }

  ~ScopedDirectory() { (void)rmdir(path_.c_str()); }

  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

int Connect(const std::string& path) {
  const int descriptor = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (descriptor < 0) throw std::runtime_error("client socket creation failed");
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1U);
  if (connect(descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    (void)close(descriptor);
    throw std::runtime_error("client socket connection failed");
  }
  return descriptor;
}

void SendAll(int descriptor, const std::string& bytes) {
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t sent = send(descriptor, bytes.data() + offset, bytes.size() - offset, 0);
    if (sent > 0) {
      offset += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent < 0 && errno == EINTR) continue;
    throw std::runtime_error("coalesced control frames were not sent");
  }
}

}  // namespace

int main() {
  ScopedDirectory directory;
  const std::string token(64U, 'a');
  const InteractiveAuthentication authentication{
      getpid(), geteuid(), directory.path() + "/command.sock", token};
  InteractiveSocketServer server(authentication);
  const int client = Connect(authentication.socket_path.string());
  try {
    const std::string stop = "{\"kind\":\"stop\",\"token\":\"" + token +
        "\",\"reason\":\"test\"}\n";
    const std::string later_target = "{\"kind\":\"target\",\"token\":\"" + token +
        "\",\"linear_velocity_m_s\":1,\"angular_velocity_rad_s\":0}\n";
    SendAll(client, stop.substr(0, stop.size() - 1U));
    std::vector<std::string> messages;
    Require(server.Poll(&messages) == InteractiveSocketServer::Event::kNone && messages.empty(),
            "incomplete authenticated stop must not be terminal before newline");

    SendAll(client, stop.substr(stop.size() - 1U) + "{bad-json\n" + later_target);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    InteractiveSocketServer::Event event = InteractiveSocketServer::Event::kNone;
    while (std::chrono::steady_clock::now() < deadline) {
      event = server.Poll(&messages);
      if (event == InteractiveSocketServer::Event::kStop) break;
      if (poll(nullptr, 0, 1) < 0 && errno != EINTR) {
        throw std::runtime_error("bounded socket poll failed");
      }
    }
    Require(event == InteractiveSocketServer::Event::kStop,
            "authenticated stop before malformed bytes did not arrive before deadline");
    Require(messages.size() == 1U && messages.front() == stop.substr(0, stop.size() - 1U),
            "terminal socket result retained bytes after stop");
  } catch (...) {
    (void)close(client);
    throw;
  }
  (void)close(client);
  return 0;
}
