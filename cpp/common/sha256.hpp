// 阶段四 C++ 公共内部工具：OpenSSL EVP 的严格 SHA-256 字节摘要。
#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string_view>

#include <openssl/evp.h>

namespace stage4 {

inline std::array<std::byte, 32> Sha256(std::string_view payload) {
  EVP_MD_CTX* const context = EVP_MD_CTX_new();
  if (context == nullptr) {
    throw std::runtime_error("SHA-256 context allocation failed");
  }
  std::array<std::byte, 32> digest{};
  unsigned int size = 0;
  const bool valid =
      EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1 &&
      EVP_DigestUpdate(context, payload.data(), payload.size()) == 1 &&
      EVP_DigestFinal_ex(
          context, reinterpret_cast<unsigned char*>(digest.data()), &size) == 1 &&
      size == digest.size();
  EVP_MD_CTX_free(context);
  if (!valid) {
    throw std::runtime_error("SHA-256 calculation failed");
  }
  return digest;
}

inline std::string Bytes(const std::array<std::byte, 32>& digest) {
  return std::string(reinterpret_cast<const char*>(digest.data()), digest.size());
}

}  // namespace stage4
