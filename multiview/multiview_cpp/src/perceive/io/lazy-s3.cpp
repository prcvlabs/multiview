
#include "lazy-s3.hpp"
#include "s3.hpp"

#include "perceive/utils/file-system.hpp"
#include "perceive/utils/string-utils.hpp"

namespace perceive
{
template<typename T>
static void write_to_lazy_cache(const string_view fname,
                                const T& data) noexcept(false)
{
   const auto dname = dirname(fname);
   if(!is_directory(dname))
      if(!mkdir_p(dname))
         throw std::runtime_error(format(
             "could not make lazy-cache directory '{}' for file '{}'",
             dname,
             fname));
   file_put_contents(fname, data);
}

// ------------------------------------------------------------------- lazy-load
//
template<typename T>
void lazy_load_T(const string_view path, T& out) noexcept(false)
{
   if(is_s3_uri(path))
      lazy_s3_load(path, out);
   else
      file_get_contents(path, out);
}

void lazy_load(const string_view path, string& out) noexcept(false)
{
   lazy_load_T(path, out);
}

void lazy_load(const string_view path, vector<char>& out) noexcept(false)
{
   lazy_load_T(path, out);
}

// ------------------------------------------------------------------ lazy-store
//
template<typename T>
void lazy_store_T(const string_view path, T& out) noexcept(false)
{
   if(is_s3_uri(path))
      lazy_s3_store(path, out);
   else
      file_put_contents(path, out);
}

void lazy_store(const string_view path, string& out) noexcept(false)
{
   lazy_store_T(path, out);
}

void lazy_store(const string_view path, vector<char>& out) noexcept(false)
{
   lazy_store_T(path, out);
}

// ----------------------------------------------------------------- lazy-exists

LazyExists lazy_exists(const string_view path) noexcept
{
   if(path.empty()) return LazyExists::NOWHERE;
   if(!is_s3_uri(path))
      return is_regular_file(path.data()) ? LazyExists::FILESYSTEM
                                          : LazyExists::NOWHERE;

   const bool is_cache = is_regular_file(lazy_s3_cache_filename(path));
   if(is_cache) return LazyExists::IN_CACHE;

   bool is_s3 = false;
   try {
      is_s3 = s3_exists(path);
   } catch(std::runtime_error& e) {
      // do nothing
   }
   if(is_s3) return LazyExists::S3_ONLY;

   return LazyExists::NOWHERE;
}

// ---------------------------------------------------------------- lazy-s3-load
template<typename T>
void lazy_s3_load_T(const string_view s3_path, T& out) noexcept(false)
{
   const auto fname = lazy_s3_cache_filename(s3_path);

   if(is_regular_file(fname)) {
      // TRACE(format("lazy-s3 loaded from cache: '{}'", s3_path));
      try {
         file_get_contents(fname, out);
         return;
      } catch(std::exception&) {
         // We're fine so far, just fall back on s3
      }
      // TRACE(format("lazy-s3 from cache failed: '{}'", s3_path));
   }

   // TRACE(format("lazy-s3 loaded from s3: '{}'", s3_path));
   s3_load(s3_path, out);
   // TRACE(format("lazy-s3 storing in cache: '{}'", fname));
   write_to_lazy_cache(fname, out);
}

void lazy_s3_load(const string_view s3_path, string& out) noexcept(false)
{
   lazy_s3_load_T(s3_path, out);
}
void lazy_s3_load(const string_view s3_path, vector<char>& out) noexcept(false)
{
   lazy_s3_load_T(s3_path, out);
}

// --------------------------------------------------------------- lazy-s3-store

template<typename T>
void lazy_s3_store_T(const string_view s3_path, T& data) noexcept(false)
{
   const auto fname = lazy_s3_cache_filename(s3_path);
   // TRACE(format("lazy-s3 uploading to s3: '{}'", s3_path));
   s3_store(s3_path, data);
   // TRACE(format("lazy-s3 storing in cache: '{}'", fname));
   write_to_lazy_cache(fname, data);
}

void lazy_s3_store(const string_view s3_path,
                   const string& data) noexcept(false)
{
   lazy_s3_store_T(s3_path, data);
}

void lazy_s3_store(const string_view s3_path,
                   const vector<char>& data) noexcept(false)
{
   lazy_s3_store_T(s3_path, data);
}

// -------------------------------------------------------------- lazy-s3-exists

LazyExists lazy_s3_exists(const string_view path) noexcept
{
   return is_s3_uri(path) ? lazy_exists(path) : LazyExists::NOWHERE;
}

// ------------------------------------------------------ lazy-s3-cache-filename
string lazy_s3_cache_filename(const string_view s3_path) noexcept
{
   static const std::string prefix = "s3://"s;
   if(!is_s3_uri(s3_path))
      FATAL(format("expected s3 uri, but got: '{}'", s3_path));

   return format(
       "{}/{}",
       multiview_lazy_s3_dir(),
       string_view(&s3_path[prefix.size()], s3_path.size() - prefix.size()));
}

} // namespace perceive
