// 阶段四 C1：供 Python/C++ golden 调用的五 topic 原始 payload SDK 验证工具。
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "../common/sha256.hpp"
#include "slope_sim/client/raw_v2_payload.hpp"

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
  if (argc != 7 || std::string(argv[1]) != "--topic" ||
      std::string(argv[3]) != "--descriptor-set" || std::string(argv[5]) != "--payload") {
    std::cerr << "error: usage: --topic NAME --descriptor-set ABSOLUTE_PATH --payload ABSOLUTE_PATH\n";
    return 64;
  }
  try {
    const std::string descriptor = ReadInputFile(argv[4]);
    const std::string payload = ReadInputFile(argv[6]);
    switch (slope_sim::client::v2::ValidateRawV2Payload(
        argv[2], payload, stage4::Bytes(stage4::Sha256(descriptor)))) {
      case slope_sim::client::v2::RawV2PayloadValidation::kValid:
        std::cout << "valid\n";
        return 0;
      case slope_sim::client::v2::RawV2PayloadValidation::kUnknownTopic:
        std::cerr << "error: topic is not a frozen v2 contract\n";
        return 66;
      case slope_sim::client::v2::RawV2PayloadValidation::kDescriptorDigestMismatch:
        std::cerr << "error: descriptor digest differs from descriptor set\n";
        return 66;
      case slope_sim::client::v2::RawV2PayloadValidation::kInvalidPayload:
        std::cerr << "error: payload does not match the v2 topic type\n";
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
