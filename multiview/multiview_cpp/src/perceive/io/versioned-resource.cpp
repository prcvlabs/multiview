
#include "versioned-resource.hpp"

#include <cstdio>
#include <regex>
#include <string>

#include "perceive/utils/md5.hpp"

#define This VersionedResource

namespace perceive
{
VersionedResource This::make(const string_view in_s) noexcept(false)
{
   VersionedResource out;

   string s = in_s.data();

   auto pop_underscores = [&](string& s) {
      while(s.size() > 0 && s.back() == '_') s.pop_back();
   };
   pop_underscores(s);

   out.digest = extract_md5_hexdigest(s);
   if(!out.digest.empty()) {
      const auto pos = s.find(out.digest);
      Expects(pos != string::npos);
      s.replace(pos, out.digest.size(), ""s);
      pop_underscores(s);
   }

   // Extract the version number
   {
      static const std::regex re = std::regex("([0-9]+)v_");
      auto r                     = s;
      std::reverse(begin(r), end(r));
      std::smatch match;
      if(std::regex_search(r, match, re)) {
         string m = match.str();
         std::reverse(begin(m), end(m));
         Expects(m.size() > 2);
         out.version = atoi(&m[2]);

         const auto pos = s.find(m);
         Expects(pos != string::npos);
         s.replace(pos, m.size(), ""s);
         pop_underscores(s);
      }
   }

   bool is_version_number = true;
   auto pos               = s.find_last_of("_v");
   if(pos == string::npos or pos == 0) {
   } else {
      for(auto i = pos + 2; i < s.size() && is_version_number; ++i)
         if(!std::isdigit(s[i])) is_version_number = false;
      s = s.substr(0, pos - 1);
   }

   pop_underscores(s);
   out.name = std::move(s);

   return out;
}

string This::to_string() const noexcept(false)
{
   return format(
       "name = '{}', digest = {}, version = {}", name, digest, version);
}

string str(const VersionedResource& o) noexcept(false) { return o.to_string(); }

} // namespace perceive
