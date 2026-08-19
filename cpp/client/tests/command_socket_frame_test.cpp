// runSim v2：无 eCAL 条件下验证 Unix stream 的 NDJSON 分帧边界。
#include "slope_sim/client/command_socket_framer.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::string Frame(std::string payload, std::size_t bytes = 0U) {
  if (bytes == 0U) return payload + "\n";
  Require(payload.size() + 1U <= bytes, "test payload exceeds requested frame size");
  payload.append(bytes - payload.size() - 1U, ' ');
  payload.push_back('\n');
  return payload;
}

}  // namespace

int main() {
  using slope_sim::client::v2::CommandSocketFramer;
  CommandSocketFramer framer;
  std::vector<std::string> frames;
  const std::string first = Frame("{\"kind\":\"status\",\"state\":\"ready\"}");
  const std::string second = Frame("{\"kind\":\"status\",\"state\":\"active\"}");

  Require(framer.Append(first.substr(0, 9U), &frames) == CommandSocketFramer::Result::kIncomplete,
          "split prefix must remain incomplete");
  Require(frames.empty(), "split prefix produced a frame");
  Require(framer.Append(first.substr(9U) + second, &frames) == CommandSocketFramer::Result::kFrame,
          "coalesced suffix was not accepted");
  Require(frames.size() == 2U, "coalesced stream did not produce two ordered frames");
  Require(frames[0] == first.substr(0, first.size() - 1U), "first frame order differs");
  Require(frames[1] == second.substr(0, second.size() - 1U), "second frame order differs");

  frames.clear();
  const std::string stop = Frame("{\"kind\":\"stop\"}");
  const std::string target = Frame("{\"kind\":\"target\"}");
  Require(framer.Append(stop + target, &frames) == CommandSocketFramer::Result::kFrame,
          "coalesced stop and target were not framed");
  Require(frames.size() == 2U, "coalesced stop and target frame count differs");
  bool has_target = true;
  int target_renewals = 0;
  const bool terminal = slope_sim::client::v2::ProcessCommandSocketFramesUntilTerminal(
      frames, [&has_target, &target_renewals, &stop, &target](const std::string& frame) {
        if (frame == stop.substr(0, stop.size() - 1U)) {
          has_target = false;
          return true;
        }
        Require(frame == target.substr(0, target.size() - 1U), "unexpected coalesced control frame");
        has_target = true;
        ++target_renewals;
        return false;
      });
  const std::vector<float> zero_command(2U, 0.0F);
  const std::vector<float> chosen_command = has_target ? std::vector<float>{1.0F, 1.0F} : zero_command;
  Require(terminal, "authenticated stop did not terminate its coalesced batch");
  Require(target_renewals == 0, "target after stop renewed the command lease");
  Require(chosen_command == zero_command, "stop batch did not choose a zero command");

  frames.clear();
  const std::string malformed = Frame("{bad-json");
  Require(framer.Append(stop + malformed, &frames) == CommandSocketFramer::Result::kFrame,
          "coalesced stop and malformed frame were not framed");
  Require(frames.size() == 2U, "coalesced stop and malformed frame count differs");
  int frames_parsed_after_stop = 0;
  const bool malformed_terminal = slope_sim::client::v2::ProcessCommandSocketFramesUntilTerminal(
      frames, [&frames_parsed_after_stop, &stop](const std::string& frame) {
        if (frame == stop.substr(0, stop.size() - 1U)) return true;
        ++frames_parsed_after_stop;
        throw std::runtime_error("frame after authenticated stop must not be parsed");
      });
  Require(malformed_terminal, "authenticated stop did not terminate malformed batch");
  Require(frames_parsed_after_stop == 0, "malformed frame after stop was parsed");

  frames.clear();
  const std::string exact = Frame("{}", 1024U);
  Require(framer.Append(exact, &frames) == CommandSocketFramer::Result::kFrame,
          "1024-byte frame was rejected");
  Require(frames.size() == 1U && frames.front().size() == 1023U, "1024-byte frame payload differs");

  frames.clear();
  const std::string oversized = Frame("{}", 1025U);
  Require(framer.Append(oversized, &frames) == CommandSocketFramer::Result::kOversize,
          "1025-byte frame was accepted");
  Require(frames.empty(), "oversized frame yielded a message");
  return 0;
}
