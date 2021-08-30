
#pragma once

#include <string>

#include "s3.hpp"

namespace perceive
{
enum class LazyExists : int {
   IN_CACHE,
   S3_ONLY,    // On s3, but NOT in cache
   FILESYSTEM, // it's a regular file, OUTSIDE of the cache folder
   NOWHERE
};

// Will attempt a lazy-s3-load if an s3-uri, otherwise loads local file-system
void lazy_load(const string_view path, string& out) noexcept(false);
void lazy_load(const string_view path, vector<char>& out) noexcept(false);
void lazy_store(const string_view path, string& out) noexcept(false);
void lazy_store(const string_view path, vector<char>& out) noexcept(false);
LazyExists lazy_exists(const string_view path) noexcept;

void lazy_s3_load(const string_view s3_path, string& out) noexcept(false);
void lazy_s3_load(const string_view s3_path, vector<char>& out) noexcept(false);

void lazy_s3_store(const string_view s3_path,
                   const string& data) noexcept(false);
void lazy_s3_store(const string_view s3_path,
                   const vector<char>& data) noexcept(false);
LazyExists
lazy_s3_exists(const string_view path) noexcept; // never returns FILESYSTEM

// Returns an empty string if 's3-path' isn't a valid s3 URI
string lazy_s3_cache_filename(const string_view s3_path) noexcept;

} // namespace perceive
