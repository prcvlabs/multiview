
#pragma once

#include <cstddef>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

namespace perceive
{
void ensure_aws_is_init();

void s3_load(const string_view s3_path, std::string& out) noexcept(false);
void s3_load(const string_view s3_path, std::vector<char>& out) noexcept(false);

void s3_store(const string_view s3_path, const string& in) noexcept(false);
void s3_store(const string_view s3_path,
              const std::vector<char>& in) noexcept(false);

bool s3_exists(const string_view s3_path) noexcept(false);

void s3_remove(const string_view s3_path) noexcept(false);

bool is_s3_uri(const string_view uri) noexcept;

string s3_dirname(const string_view uri) noexcept;

} // namespace perceive
