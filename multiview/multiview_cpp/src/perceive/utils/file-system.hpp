
#pragma once

#include <fstream>
#include <iterator>
#include <string>
#include <string_view>

namespace perceive
{
using std::error_code;

// ------------------------------------------------------- file-get/put-contents

error_code file_get_contents(const std::string_view fname,
                             std::string& out) noexcept;
error_code file_get_contents(const std::string_view fname,
                             std::vector<char>& out) noexcept;

inline std::string
file_get_contents(const std::string_view fname) noexcept(false)
{
   std::string out;
   file_get_contents(fname, out);
   return out;
}

error_code file_put_contents(const std::string_view fname,
                             const std::string_view dat) noexcept;
error_code file_put_contents(const std::string_view fname,
                             const std::vector<char>& dat) noexcept;

inline void file_put_contents(const std::string_view fname,
                              const char* dat) noexcept(false)
{
   std::ofstream(fname.data(), std::ios::binary) << dat;
}

inline void file_append_contents(const std::string_view fname,
                                 const std::string_view data) noexcept(false)
{
   auto ifs = std::ofstream(fname.data(), std::ios::app | std::ios::binary);
   ifs.write(data.data(), std::streamsize(data.size()));
}

inline void file_append_contents(const std::string_view fname,
                                 const std::vector<char>& data) noexcept(false)
{
   file_append_contents(fname, std::string_view(&data[0], data.size()));
}

// ----------------------------------------------------------- is-file/directory

bool is_regular_file(const std::string_view filename) noexcept;
bool is_directory(const std::string_view filename) noexcept;

// -------------------------------------------------------- is-readable/writable

bool is_readable_file(const char* filename) noexcept;
inline bool is_readable_file(const std::string& filename) noexcept
{
   return is_readable_file(filename.c_str());
}

bool is_writeable_file(const char* filename) noexcept;
inline bool is_writeable_file(const std::string& filename) noexcept
{
   return is_writeable_file(filename.c_str());
}

// --------------------------------------------------- basename/dirname/file_ext

std::string absolute_path(const std::string_view filename) noexcept;
std::string basename(const std::string_view filename,
                     const bool strip_extension = false) noexcept;
std::string dirname(const std::string_view filename) noexcept;
std::string_view dirname_sv(const std::string_view filename) noexcept;
std::string file_ext(const std::string_view filename) noexcept; // like ".png"
std::string extensionless(const std::string_view filename) noexcept;

// ----------------------------------------------------------------------- mkdir

bool mkdir(const std::string_view dname) noexcept;
bool mkdir_p(const std::string_view dname) noexcept;

// --------------------------------------------------------- make-temp-directory
// make_temp_directory("/tmp/fooXXXXXX");
std::string make_temp_directory(const string_view p) noexcept(false);

// ------------------------------------------------------------------ remove-all

std::error_code delete_file(const string_view path);

int remove_all(const string_view path) noexcept(false);

// Recursively remove all files where 'match(filename)' is true
int remove_all(const string_view directory,
               std::function<bool(const string_view)> match) noexcept;

} // namespace perceive
