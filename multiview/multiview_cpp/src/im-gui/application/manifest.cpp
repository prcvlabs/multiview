
#include "manifest.hpp"

#include "config.hpp"

#include "perceive/utils/file-system.hpp"

#include <filesystem>
#include <regex>

#define This Manifest

namespace perceive::gui
{
// Predefinitions
static std::regex make_glob(const string& pattern);

// ------------------------------------------------------------------- make-glob
//
static std::regex make_glob(string_view pattern)
{
   int pos = 0;

   vector<char> buffer_memory(pattern.size() * 2 + 1);
   char* buffer    = &buffer_memory[0];
   const char* src = pattern.data();

   while(*src) {
      switch(*src) {
      case '\\':
         buffer[pos++] = '\\';
         buffer[pos++] = '\\';
         break;
      case '^':
         buffer[pos++] = '\\';
         buffer[pos++] = '^';
         break;
      case '.':
         buffer[pos++] = '\\';
         buffer[pos++] = '.';
         break;
      case '$':
         buffer[pos++] = '\\';
         buffer[pos++] = '$';
         break;
      case '|':
         buffer[pos++] = '\\';
         buffer[pos++] = '|';
         break;
      case '(':
         buffer[pos++] = '\\';
         buffer[pos++] = '(';
         break;
      case ')':
         buffer[pos++] = '\\';
         buffer[pos++] = ')';
         break;
      case '[':
         buffer[pos++] = '\\';
         buffer[pos++] = '[';
         break;
      case ']':
         buffer[pos++] = '\\';
         buffer[pos++] = ']';
         break;
      case '+':
         buffer[pos++] = '\\';
         buffer[pos++] = '+';
         break;
      case '/':
         buffer[pos++] = '\\';
         buffer[pos++] = '/';
         break;
      case '?': buffer[pos++] = '.'; break;
      case '*':
         buffer[pos++] = '.';
         buffer[pos++] = '*';
         break;
      default: buffer[pos++] = *src;
      }
      src++;
   }
   buffer[pos++] = '\0';

   return std::regex(buffer);
}

// ----------------------------------------------------------- directory-recurse
//
static void
recursive_directory_pass(string_view directory,
                         std::function<void(string_view)> f) noexcept
{
   namespace fs = std::filesystem;
   const string dir(begin(directory), end(directory));
   const auto opts = fs::directory_options::follow_directory_symlink;
   for(auto& file : fs::recursive_directory_iterator(dir, opts))
      if(fs::is_regular_file(file)) f(file.path().string());
}

// --------------------------------------------------------------- make-manifest
//
Manifest make_manifest(const Config& config) noexcept
{
   constexpr string_view mp4_ext        = ".mp4";
   constexpr string_view annotation_ext = ".annotations";

   vector<string> mp4s        = {};
   vector<string> annotations = {};

   Manifest o = {};

   std::stringstream ss{""};

   auto push_err = [&](string_view msg) {
      LOG_ERR(format("{}", msg));
      ss << msg << '\n';
      o.has_error = true;
   };

   recursive_directory_pass(config.annotation_dir, [&](string_view fname) {
      if(ends_with(fname, mp4_ext))
         mp4s.emplace_back(begin(fname), end(fname));
      else if(ends_with(fname, annotation_ext))
         annotations.emplace_back(begin(fname), end(fname));
   });

   // Okay, we should have 1 annotations file...
   if(annotations.size() == 0) {
      push_err(format(
          "failed to find any file with extension '{}' in directory '{}'",
          annotation_ext,
          config.annotation_dir));
   } else if(annotations.size() == 1) {
      o.annotation_fname = annotations.front();
   } else {
      push_err(format("found multiple '{}' files in directory '{}': [{}]",
                      annotation_ext,
                      config.annotation_dir,
                      implode(cbegin(annotations), cend(annotations), ", ")));
   }

   // Okay, is we have an annotation fname, try to find the mp4 file that goes
   // with it...
   if(annotations.size() == 1) {
      const auto val = format(
          "{}{}", ::perceive::basename(o.annotation_fname, true), mp4_ext);
      auto ii = std::find_if(cbegin(mp4s), cend(mp4s), [&](const auto& fname) {
         return ends_with(fname, val);
      });
      if(ii == cend(mp4s)) {
         push_err(format("failed to find '{}' in directory '{}'",
                         val,
                         config.annotation_dir));
      } else {
         o.mp4_fname = *ii;
      }
   }

   o.err_message = ss.str();

   return o;
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   return format(R"V0G0N(
Manifest:
   annotation-fname:   {}
   mp4-fname:          {}
   has-error:          {}
   err-messg:          {}
)V0G0N",
                 annotation_fname,
                 mp4_fname,
                 str(has_error),
                 err_message);
}

} // namespace perceive::gui

#undef This
