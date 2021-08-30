
#include "versioned-filename.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/utils/file-system.hpp"

#define This VersionedFilename

namespace perceive
{
// ------------------------------------------------------------------------ init
//
void This::init(const string_view fname) noexcept
{
   // this->directory = ::perceive::dirname(string(fname));
   // this->basename  = ::perceive::basename(fname, true);
   // this->extension = file_ext(fname);
   // { // Try to find a version number
   //    const auto pos = basename.find_last_of("_v"s);
   //    this->version  = -1;
   //    if(pos != string::npos) {
   //       this->filestem = this->basename.substr(0, pos - 1);
   //       char* end      = nullptr;
   //       int v          = std::strtol(&fname[pos + 1], &end, 10);
   //       if(*end == '\0') this->version = v;
   //    } else {
   //       this->filestem = this->basename;
   //    }
   // }
}

// --------------------------------------------------------------------- setters
//
void This::set_directory(const string_view s) noexcept { directory_ = s; }

void This::set_stem(const string_view s) noexcept(false)
{
   auto ii = std::find_if(cbegin(s), cend(s), [](const char ch) {
      return !std::isprint(ch) or (ch == '/');
   });
   if(ii != cend(s))
      throw std::runtime_error(
          format("invalid character '{:c}' found in stem '{}'", *ii, s));
   stem_part_ = s;
}

void This::set_extension(const string_view s) noexcept(false)
{
   if(s.empty()) {
      extension_ = s;
   } else if(s[0] != '.') {
      throw std::runtime_error(format("extension '{}' must begin with '.'", s));
   } else {
      extension_ = s;
   }
}

// ------------------------------------------------------------------------ date
//
void This::set_date(const Timestamp ts) noexcept { date_part_ = str(ts); }
void This::set_date(int year, int month, int day) noexcept(false)
{
   if(!is_valid_date(year, month, day))
      throw std::runtime_error(
          format("invalid year/month/day: {:4d}-{:2d}-{:2d}", year, month, day));
   date_part_ = format("{:4d}-{:2d}-{:2d}", year, month, day);
}
void This::clear_date() noexcept { date_part_.clear(); }

// -------------------------------------------------------------------- filename

static std::stringstream& ss_basename(const This& vfname,
                                      std::stringstream& ss) noexcept
{
   bool place_underscore = false;
   auto push_part        = [&](const string_view s) {
      if(!s.empty()) {
         if(place_underscore) ss << '_';
         ss << s;
         place_underscore = true;
      }
   };

   push_part(vfname.stem_part());
   push_part(vfname.date_part());
   push_part(vfname.md5_part());

   return ss;
}

static std::stringstream& ss_filename(const This& vfname,
                                      std::stringstream& ss) noexcept
{
   if(!vfname.directory().empty()) ss << vfname.directory() << '/';
   ss_basename(vfname, ss);
   ss << vfname.extension();
   return ss;
}

string This::filename() const noexcept
{
   std::stringstream ss;
   return ss_filename(*this, ss).str();
}

string This::basename() const noexcept
{
   std::stringstream ss;
   return ss_basename(*this, ss).str();
}

// ------------------------------------------------------------------- to-string

string This::to_string() const noexcept
{
   return format(R"V0G0N({
   "directory": {},
   "basename":  {},
   "extension": {},
   "stem-part": {},
   "md5-part":  {},
   "date-part": {},
   "filename":  {}
}{})V0G0N",
                 json_encode(directory()),
                 json_encode(basename()),
                 json_encode(extension()),
                 json_encode(stem_part()),
                 json_encode(md5_part()),
                 json_encode(date_part()),
                 json_encode(filename()),
                 "");
}

string str(const VersionedFilename& s) noexcept { return s.to_string(); }

} // namespace perceive
