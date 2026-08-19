// 阶段四 C1：验证 Python/C++ 之间 WheelCommand 原始 bytes 的小型 SDK 工具。
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "../common/sha256.hpp"
#include "slope_sim/client/raw_wheel_command.hpp"

namespace {

namespace fs = std::filesystem;

std::string ReadInputFile(const char* raw_path) {
  const fs::path path(raw_path);
  if (!path.is_absolute() || path.lexically_normal() != path || !fs::is_regular_file(path)) {
    throw std::invalid_argument("input must be an absolute normalized regular file");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("input cannot be opened");
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 5 || std::string(argv[1]) != "--descriptor-set" ||
      std::string(argv[3]) != "--payload") {
    std::cerr << "error: usage: --descriptor-set ABSOLUTE_PATH --payload ABSOLUTE_PATH\n";
    return 64;
  }
  try {
    const std::string descriptor = ReadInputFile(argv[2]);
    const std::string payload = ReadInputFile(argv[4]);
    switch (slope_sim::client::v2::ValidateRawWheelCommand(
        payload, stage4::Bytes(stage4::Sha256(descriptor)))) {
      case slope_sim::client::v2::RawWheelCommandValidation::kValid:
        std::cout << "valid\n";
        return 0;
      case slope_sim::client::v2::RawWheelCommandValidation::kDescriptorDigestMismatch:
        std::cerr << "error: descriptor digest differs from descriptor set\n";
        return 66;
      case slope_sim::client::v2::RawWheelCommandValidation::kInvalidPayload:
        std::cerr << "error: payload is not a valid WheelCommand\n";
        return 66;
    }
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 64;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  std::cerr << "error: unhandled validation state\n";
  return 1;
}
