
#pragma once

#include <string>
#include <string_view>
#include <vector>

// Checks validity
bool is_base64(const std::string_view s) noexcept;

// Decode
std::size_t base64_decode_length(const std::string_view s) noexcept(false);
std::string base64_decode(const std::string_view s) noexcept(false);

// Returns the size of the decoded data. 'buffer' MUST be big enough,
// and correctly aligned. Will always decode precisely as many bytes
// as returned by 'base64_decode_length'
void base64_decode(const std::string_view s, void* buffer) noexcept(false);

// Encode
std::size_t base64_encode_length(std::size_t sz) noexcept;
std::string base64_encode(const void* src, std::size_t sz) noexcept;
std::string base64_encode(const std::vector<char>& buf) noexcept;
std::string base64_encode(const std::string_view buf) noexcept;
