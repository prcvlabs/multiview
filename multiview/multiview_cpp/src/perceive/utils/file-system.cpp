
#include "file-system.hpp"

#include <filesystem>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace perceive
{
using std::error_code;

bool is_readable_file(const char* filename) noexcept
{
   return is_regular_file(filename) && access(filename, R_OK);
}

bool is_writeable_file(const char* filename) noexcept
{
   return is_regular_file(filename) && access(filename, W_OK);
}

bool is_regular_file(const std::string_view filename) noexcept
{
   try {
      auto p = std::filesystem::path(filename);
      return std::filesystem::is_regular_file(p);
   } catch(...) {}
   return false;
}

bool is_directory(const std::string_view filename) noexcept
{
   try {
      auto p = std::filesystem::path(filename);
      return std::filesystem::is_directory(p);
   } catch(...) {}
   return false;
}

// ----------------------------------------------------------- file-get-contents

/**
 * @private
 */
template<typename T>
error_code file_get_contentsT(const std::string_view fname, T& data) noexcept
{
   std::unique_ptr<FILE, std::function<void(FILE*)>> fp(
       fopen(fname.data(), "rb"), [](FILE* ptr) {
          if(ptr) fclose(ptr);
       });

   if(fp == nullptr) return std::make_error_code(std::errc(errno));

   if(fseek(fp.get(), 0, SEEK_END) == -1) {
      // Probably EBADF: stream was not seekaable
      return std::make_error_code(std::errc(errno));
   }

   auto fpos = ftell(fp.get());
   if(fpos == -1) {
      // Probably EBADF: stream was not seekaable
      return std::make_error_code(std::errc(errno));
   }

   auto sz = size_t(fpos < 0 ? 0 : fpos);

   try {
      data.reserve(sz);
      data.resize(sz);
   } catch(std::length_error& e) {
      return std::make_error_code(std::errc::invalid_argument);
   } catch(std::bad_alloc& e) {
      return std::make_error_code(std::errc::not_enough_memory);
   } catch(...) {
      // some other error thrown by the allocator... shouldn't happen
      return std::make_error_code(std::errc::io_error);
   }

   if(fseek(fp.get(), 0, SEEK_SET) == -1) {
      // Probably should never happen
      return std::make_error_code(std::errc(errno));
   }

   if(data.size() != fread(&data[0], 1, data.size(), fp.get())) {
      if(ferror(fp.get()))
         return std::make_error_code(std::errc(errno));
      else if(feof(fp.get()))
         return std::make_error_code(std::errc::io_error);
      else
         return std::make_error_code(std::errc::io_error);
   }

   if(FILE* ptr = fp.release(); fclose(ptr) != 0) {
      return std::make_error_code(std::errc(errno));
   }

   return {};
}

error_code file_get_contents(const std::string_view filename,
                             std::string& out) noexcept
{
   return file_get_contentsT(filename, out);
}

/**
 * @ingroup niggly-filesystem
 * @brief Overload to `file_put_contents(string_view, string)`.
 */
error_code file_get_contents(const std::string_view filename,
                             std::vector<char>& out) noexcept
{
   return file_get_contentsT(filename, out);
}

// ----------------------------------------------------------- file-put-contents

error_code file_put_contents(const std::string_view filename,
                             const std::string_view dat) noexcept
{
   FILE* fp = fopen(filename.data(), "wb");
   if(fp == nullptr) return std::make_error_code(std::errc(errno));

   error_code ec = {};

   auto sz = fwrite(dat.data(), 1, dat.size(), fp);
   if(sz != dat.size()) {
      if(ferror(fp))
         ec = std::make_error_code(std::errc(errno));
      else if(feof(fp))
         ec = std::make_error_code(std::errc::io_error);
      else
         ec = std::make_error_code(std::errc::io_error);
   }
   if(fclose(fp) != 0)
      if(!ec) ec = make_error_code(std::errc(errno));

   return ec;
}

/**
 * @ingroup niggly-filesystem
 * @brief Overload to `file_put_contents(const string_view, const string_view)`.
 */
error_code file_put_contents(const std::string_view fname,
                             const std::vector<char>& dat) noexcept
{
   return file_put_contents(fname, std::string_view(&dat[0], dat.size()));
}

// --------------------------------------------------- basename/dirname/file_ext

/**
 * @ingroup niggly-filesystem
 * @brief The absolute file path.
 */
std::string absolute_path(const std::string_view filename) noexcept
{
   std::string ret;
   try {
      namespace bf = std::filesystem;
      ret          = bf::absolute(bf::path(filename)).string();
   } catch(...) {}
   return ret;
}

/**
 * @ingroup niggly-filesystem
 * @brief Like the shell command `basename`, with the option of stripping the
 *        extension.
 */
std::string basename(const std::string_view filename,
                     const bool strip_extension) noexcept
{
   std::string ret;
   try {
      namespace bf = std::filesystem;
      if(strip_extension)
         ret = bf::path(filename).stem().string();
      else
         ret = bf::path(filename).filename().string();
   } catch(...) {}
   return ret;
}

/**
 * @ingroup niggly-filesystem
 * @brief Like the shell command `dirname`.
 */
std::string dirname(const std::string_view filename) noexcept
{
   std::string ret;
   try {
      namespace bf = std::filesystem;
      ret          = bf::path(filename).parent_path().string();
   } catch(...) {}
   return ret;
}

/**
 * @ingroup niggly-filesystem
 * @brief Returns the file extension, something like `.png`
 */
std::string file_ext(const std::string_view filename) noexcept
{
   std::string ret;
   try {
      namespace bf = std::filesystem;
      ret          = bf::path(filename).extension().string();
   } catch(...) {}
   return ret;
}

std::string extensionless(const std::string_view filename) noexcept
{
   const auto ext = file_ext(filename);
   return string(filename.data(), filename.size() - ext.size());
}

// ------------------------------------------------------------------ dirname-sv

string_view dirname_sv(const string_view filename) noexcept
{
   // foo          --> .
   // s3://        --> s3://
   // /            --> /
   // //           --> //
   // s3://foo/bar --> s3://foo
   // s3://foo     --> s3://
   // /foo         --> /

   static const char dot[]   = ".";
   auto is_colon_slash_slash = [filename](const auto pos) {
      return pos >= 0 && filename[size_t(pos)] == ':'
             && int64_t(filename.size()) > pos + 2
             && filename[size_t(pos + 1)] == '/'
             && filename[size_t(pos + 2)] == '/';
   };

   auto starts_slash_slash = [filename]() {
      return filename.size() >= 2 && filename[0] == '/' && filename[1] == '/';
   };

   if(filename.empty()) return filename;
   int64_t pos             = int64_t(filename.size()) - 1;
   bool has_trailing_slash = filename.back() == '/';

   while(pos >= 0 && filename[size_t(pos)] == '/') --pos; // skip any '/' at end
   if(pos < 0) return string_view(&filename[0], starts_slash_slash() ? 2 : 1);
   if(is_colon_slash_slash(pos))
      return string_view(&filename[0], size_t(pos + 3));

   // Search backwards for '/'
   while(pos >= 0 && filename[size_t(pos)] != '/') --pos;
   if(pos < 0) return string_view(dot); // not a single '/' character

   // Skip any consequtive '/' characters
   while(pos >= 0 && filename[size_t(pos)] == '/') --pos; // skip any '/' at end
   if(pos < 0)
      return string_view(&filename[0],
                         starts_slash_slash() ? 2 : 1); // start of the string

   if(is_colon_slash_slash(pos))
      return string_view(&filename[0], size_t(pos + 3));

   return string_view(&filename[0], size_t(pos + 1));
}

// ----------------------------------------------------------------------- mkdir
/**
 * @ingroup niggly-filesystem
 * @brief Like the shell command `mkdir`, return `true` if successful.
 */
bool mkdir(const std::string_view dname) noexcept
{
   try {
      namespace fs = std::filesystem;
      if(fs::create_directory(fs::path(dname))) return true;
   } catch(...) {}
   return false;
}

/**
 * @ingroup niggly-filesystem
 * @brief Like the shell command `mkdir -p`, return `true` if successful.
 */
bool mkdir_p(const std::string_view dname) noexcept
{
   try {
      namespace fs = std::filesystem;
      if(fs::create_directories(fs::path(dname))) return true;
   } catch(...) {}
   return false;
}

// --------------------------------------------------------- make-temp-directory

static int how_many_Xs_at_end(const char* s) noexcept
{
   int count = 0;
   int pos   = int(strlen(s) - 1);
   while(pos >= 0 and s[pos] == 'X') {
      count++;
      --pos;
   }
   return count;
}

static std::vector<char> put_6X_suffix(const string_view p)
{
   int buf_sz = int(p.size() + 6 + 1);
   std::vector<char> buf((size_t(buf_sz)));
   char* s = &buf[0];
   strcpy(s, p.data());

   // Do we end in six 'X' symbols?
   const int n_Xs = how_many_Xs_at_end(s);
   int pos        = int(p.size());
   for(int i = 0; i < 6 - n_Xs; ++i) {
      Expects(pos < buf_sz);
      s[pos++] = 'X';
   }
   Expects(pos < buf_sz);
   s[pos++] = '\0';
   return buf;
}

std::string make_temp_directory(const string_view p) noexcept(false)
{
   auto s = put_6X_suffix(p);
   if(mkdtemp(&s[0]) == nullptr)
      throw std::runtime_error("failed to create temporary directory");
   return string{&s[0]};
}

// ------------------------------------------------------------------ remove-all

std::error_code delete_file(const string_view path)
{
   namespace fs = std::filesystem;
   std::error_code ec;
   fs::remove(fs::path(path.data()), ec);
   return ec;
}

int remove_all(const string_view path) noexcept(false)
{
   namespace fs = std::filesystem;
   std::error_code ec;
   int val = int(fs::remove_all(fs::path(path.data()), ec));
   if(ec)
      throw std::runtime_error(format(
          "failed to remove directory '{}': {}\n", path, ec.message().c_str()));
   return val;
}

int remove_all(const string_view directory,
               std::function<bool(const string_view)> match) noexcept
{
   namespace fs = std::filesystem;
   std::deque<string> to_delete;

   try {
      for(auto& f : fs::recursive_directory_iterator(directory.data())) {
         if(fs::is_regular_file(f.path())) {
            auto s = f.path().string();
            if(match(s)) to_delete.push_back(std::move(s));
         }
      }

      for(auto& fname : to_delete) unlink(fname.c_str());
   } catch(...) {
      // do nothing
   }

   return int(to_delete.size());
}

} // namespace perceive
