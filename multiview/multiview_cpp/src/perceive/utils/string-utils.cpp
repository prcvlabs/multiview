
#include "string-utils.hpp"

namespace perceive
{
// ------------------------------------------------- Pretty Printing binary data

string str(const void* data, size_t sz)
{
   // 0         1         2         3         4         5         6
   // 01234567890123456789012345678901234567890123456789012345678901234567
   // 00000000: 0a23 2050 7974 686f 6e2f 432b 2b20 4d75  .# Python/C++ Mu

   auto ptr = static_cast<const char*>(data);

   auto hexit = [](int c) -> char {
      return char((c < 10) ? '0' + c : 'a' + (c - 10));
   };

   const auto row_sz = 68;
   const auto n_rows = (sz % 16 == 0) ? (sz / 16) : (1 + sz / 16);
   auto out_sz       = n_rows * row_sz;
   string out(out_sz, ' ');

   char buffer[32];
   auto process_row = [&](const auto row_number) {
      const auto row_pos = row_number * row_sz;
      snprintf(buffer, 32, "%08lx:", row_number * 16);
      std::copy(&buffer[0], &buffer[0] + 9, &out[row_pos]);
      auto pos       = row_pos + 9;
      auto ascii_pos = row_pos + 51;

      auto k = row_number * 16;
      for(auto i = k; i < k + 16 and i < sz; ++i) {
         if(i % 2 == 0) out[pos++] = ' ';
         const auto c     = ptr[i];
         out[pos++]       = hexit((c >> 4) & 0x0f);
         out[pos++]       = hexit((c >> 0) & 0x0f);
         out[ascii_pos++] = std::isprint(c) ? c : '.';
      }
      out[row_pos + 67] = '\n';
   };

   for(std::decay_t<decltype(n_rows)> row = 0; row < n_rows; ++row)
      process_row(row);

   return out;
}

// --------------------------------------------------------------- to-binary-str

std::string to_binary_str(const void* ptr, const size_t size) noexcept
{
   string s(size * 8 + (size - 1), '-');
   const uint8_t* b = reinterpret_cast<const uint8_t*>(ptr);

   size_t pos = 0;
   for(int64_t i = int64_t(size) - 1; i >= 0; --i) {
      for(int j = 7; j >= 0; --j) {
         Expects(pos < s.size());
         s[pos++] = ((b[i] >> j) & 1) ? '1' : '0';
      }
      if(i > 0) s[pos++] = '-';
   }

   Expects(pos == s.size());
   return s;
}

// --------------------------------------------------------------------- explode

vector<string> explode(const std::string_view line,
                       const std::string_view delims,
                       const bool collapse_empty_fields) noexcept(false)
{
   vector<string> o;

   if(line.empty()) return o;

   auto push_it = [&](const auto pos0, const auto pos1) {
      const auto sz = (pos1 == string::npos)
                          ? (string::size_type(line.size()) - pos0)
                          : (pos1 - pos0);
      const auto s  = string_view(&line[pos0], sz);
      if(!s.empty() or !collapse_empty_fields)
         o.push_back(string(cbegin(s), cend(s)));

      return (pos1 == string::npos) ? string::npos : pos1 + 1;
   };

   string::size_type pos = 0;
   while(pos != string::npos) {
      const auto new_pos = line.find_first_of(delims.data(), pos);
      pos                = push_it(pos, new_pos);
   }

   if(collapse_empty_fields) {
      Expects(std::all_of(
          cbegin(o), cend(o), [](const auto& s) { return !s.empty(); }));
   }

   return o;
}

// -------------------------------------------------------- substitute variables

string substitute_variables(const std::unordered_map<string, string>& vars,
                            const string& line_s)
{
   auto pos = line_s.find('$');
   if(pos == string::npos) return line_s;

   std::string_view line(line_s);
   const auto len = line.size();

   std::stringstream ss("");
   ss << line.substr(0, pos);

   auto process_variable = [&](size_t& i) {
      assert(line[i] == '$');
      if(++i == len) return;
      if(line[i] != '{') {
         ss << '$';
         ss << line[i];
         return;
      }

      auto pos = line.find('}', i);
      if(pos == string::npos)
         throw std::runtime_error("parse error reading variable name, "
                                  "missing '}'");

      string variable(line.substr(i + 1, pos - i - 1));
      const auto value = vars.find(variable);
      if(value == cend(vars))
         throw std::runtime_error(format("variable '{}' not found", variable));
      ss << value->second;
      i = pos;
   };

   for(size_t i = pos; i < len; ++i) {
      if(line[i] == '$')
         process_variable(i);
      else
         ss << line[i];
   }

   return ss.str();
}

// ------------------------------------------------------------------ string-is?

bool string_is_uppercase(const std::string_view s) noexcept
{
   for(auto& c : s)
      if(std::islower(static_cast<unsigned char>(c))) return false;
   return true;
}

string string_to_uppercase(const std::string_view s) noexcept
{
   string o{s};
   for(auto& c : o) c = char(std::toupper(c));
   return o;
}

// ----------------------------------------------------------------- str replace

string str_replace(const string_view search,
                   const string_view replace,
                   const string_view subject) noexcept
{
   if(subject.empty() or search.empty())
      return string(subject.data(), subject.size());

   std::stringstream ss{""};

   const size_t search_sz  = search.size();
   const size_t subject_sz = subject.size();

   if(subject_sz < search_sz) return string(subject.data(), subject.size());

   size_t pos = 0, last_pos = 0;
   while(pos <= subject_sz - search_sz) {
      auto ii = subject.find(search, pos);
      if(ii == string_view::npos) break;
      ss << string_view(&subject[last_pos], ii - last_pos) << replace;
      pos      = ii + search_sz;
      last_pos = pos;
   }

   if(last_pos < subject_sz) ss << string_view(&subject[last_pos]);

   return ss.str();
}

// --------------------------------------------------------------- mem-usage-str

string mem_usage_str(size_t sz) noexcept(false)
{
   size_t rem = 0;
   if(sz < 1024) return format("{} bytes", sz);

   rem = sz % 1024;
   sz  = sz / 1024;
   if(sz < 1024) return format("{}.{:03f} k", sz, real(rem) / 1024.0);

   rem = sz % 1024;
   sz  = sz / 1024;
   if(sz < 1024) return format("{}.{:03f} M", sz, real(rem) / 1024.0);

   rem = sz % 1024;
   sz  = sz / 1024;
   return format("{}.{:03f} G", sz, real(rem) / 1024.0);
}

// ------------------------------------------------------------ encode-dot-label

string encode_dot_label(const string_view s) noexcept
{
   std::stringstream ss("");
   for(auto c : s) {
      switch(c) {
      case '"': ss << "\\\""; break;
      case '\\': ss << "\\\\"; break;
      case '\n': ss << "\\n"; break;
      case ' ': ss << "\\ "; break;
      case '#': ss << "\\#"; break;
      case ',': ss << "\\,"; break;
      case '=': ss << "\\="; break;
      case '[': ss << "\\["; break;
      case ']': ss << "\\]"; break;
      case '(': ss << "\\("; break;
      case ')': ss << "\\)"; break;
      case '{': ss << "\\{"; break;
      case '}': ss << "\\}"; break;
      case '-': ss << c; break;
      case '_': ss << c; break;
      case ':': ss << "\\:"; break;
      case '.': ss << "\\."; break;
      default:
         if(!std::isalnum(c))
            FATAL(format("invalid character '{}' in graph-label '{}'", c, s));
         ss << c;
      }
   }

   return ss.str();
}

} // namespace perceive
