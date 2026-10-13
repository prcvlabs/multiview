
#pragma once

#include <cstddef>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

namespace perceive
{
// How the S3 client reaches an endpoint other than AWS (Cloudflare R2, MinIO)
struct S3Settings
{
   string endpoint              = ""s; // "" => the AWS default
   string region                = ""s;
   bool use_https               = false;
   bool path_style              = false; // bucket in the path, not the host
   bool checksums_when_required = false;
   bool disable_imds            = false;
};

// An empty `endpoint` gives the AWS defaults. Otherwise the region is "auto"
// when empty (R2), the scheme comes from the URI, checksums are calculated only
// when required, and the EC2 metadata service is never asked.
S3Settings s3_settings_for(const string_view endpoint,
                           const string_view region,
                           const bool path_style) noexcept;

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
