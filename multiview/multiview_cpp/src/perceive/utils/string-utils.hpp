
#pragma once

#include <cfloat>
#include <charconv>
#include <ctime>
#include <memory>
#include <string_view>
#include <unordered_map>

namespace perceive
{
using std::string;
using std::string_view;

// -- Terminal Colours

#define ANSI_COLOUR_RED "\x1b[31m"
#define ANSI_COLOUR_GREEN "\x1b[32m"
#define ANSI_COLOUR_YELLOW "\x1b[33m"
#define ANSI_COLOUR_BLUE "\x1b[34m"
#define ANSI_COLOUR_MAGENTA "\x1b[35m"
#define ANSI_COLOUR_CYAN "\x1b[36m"
#define ANSI_COLOUR_GREY "\x1b[37m"
#define ANSI_COLOUR_WHITE "\x1b[97m"

#define ANSI_COLOUR_RED_BG "\x1b[41m"
#define ANSI_COLOUR_GREEN_BG "\x1b[42m"
#define ANSI_COLOUR_YELLOW_BG "\x1b[43m"
#define ANSI_COLOUR_BLUE_BG "\x1b[44m"

#define ANSI_DIM "\x1b[2m"
#define ANSI_UNDERLINE "\x1b[4m"

#define ANSI_COLOUR_RESET "\x1b[0m"

// ---------------------------------------------------------------- lexical-cast
//
template<std::arithmetic I>
std::error_code
lexical_cast(std::string_view s, I& value, int base = 10) noexcept
{
   if constexpr(std::is_integral<I>::value) {
      const auto [ptr, ec] = std::from_chars(begin(s), end(s), value, base);
      return std::make_error_code(ec);
   } else {
      const auto [ptr, ec] = std::from_chars(begin(s), end(s), value);
      return std::make_error_code(ec);
   }
}

// -------------------------------------------------------------------- str shim

// Helper function
namespace detail
{
   template<typename T> string format_(const char* fmt, const T& v)
   {
      constexpr int k_size = 32;
      char buffer[k_size];
      int written = snprintf(buffer, k_size, fmt, v);
      if(written < k_size - 1) return string(buffer);
      // We never do this in our str(...) functions
      std::unique_ptr<char[]> b2(new char[size_t(written + 1)]);
      snprintf(b2.get(), size_t(written + 1), fmt, v);
      return string(b2.get());
   }

} // namespace detail

// strings
inline string& str(string& s) { return s; }
inline const string& str(const string& s) { return s; }
// inline string str(const char* s) { return string(s); } // Not necessary

// Basic types
inline string str(bool v) { return v ? "true" : "false"; }
inline string str(char c) { return string(1, c); }
inline string str(int v) { return detail::format_<int>("%d", v); }
inline string str(unsigned int v)
{
   return detail::format_<unsigned int>("%u", v);
}
inline string str(long int v) { return detail::format_<long int>("%ld", v); }
inline string str(unsigned long int v)
{
   return detail::format_<unsigned long int>("%lu", v);
}
inline string str(long long int v)
{
   return detail::format_<long long int>("%lld", v);
}
inline string str(unsigned long long int v)
{
   return detail::format_<unsigned long long int>("%llu", v);
}
inline string str(float v)
{
   return detail::format_<double>("%f", static_cast<double>(v));
}
inline string str(double v) { return detail::format_<double>("%f", v); }
inline string str(void* p) { return detail::format_<void*>("%p", p); }
inline string str(const char* p) { return string(p); }

inline string str_precise(double v)
{
   constexpr int k_size = 256;
   char buffer[k_size];
   int written = snprintf(buffer, k_size, "%.*e", DECIMAL_DIG, v);
   if(written < k_size - 1) return string(buffer);
   // We never do this in our str(...) functions
   std::unique_ptr<char[]> b2(new char[size_t(written + 1)]);
   snprintf(b2.get(), size_t(written + 1), "%.*e", DECIMAL_DIG, v);
   return string(b2.get());
}

std::string to_binary_str(const void* ptr, const size_t size) noexcept;

inline string timestamp_str(const std::time_t& t)
{
   char buf[32];
   std::strftime(buf, sizeof(buf), "%F_%T", std::localtime(&t));
   return buf;
}

template<typename T> inline string str(T* p, unsigned r, unsigned c)
{
   std::stringstream ss("");
   for(unsigned j = 0; j < r; ++j) {
      ss << "|";
      for(unsigned i = 0; i < c; ++i) {
         if(i > 0) ss << ", ";
         ss << str(p[j * r + i]);
      }
      ss << "|\n";
   }
   return ss.str();
}

// ---------------------------------------------------------------------- Indent

inline std::string indent(const string& s, int level)
{
   const std::string indent_s(size_t(level), char(' '));
   std::stringstream ss{""};
   std::istringstream input{s};
   for(string line; std::getline(input, line);)
      ss << indent_s << line << std::endl;
   string ret = ss.str();
   // Remove endl character if 's' doesn't have one
   if(s.size() > 0 && s[0] != '\n') ret.pop_back();
   return ret;
}

// ----------------------------------------------------------------- str replace

std::string str_replace(const std::string_view search,
                        const std::string_view replace,
                        const std::string_view subject) noexcept;

// --------------------------------------------------------------------- Implode

template<typename InputIt, typename F>
string implode(InputIt first, InputIt last, const std::string_view glue, F f)
{
   std::stringstream stream("");
   bool start = true;
   while(first != last) {
      if(start)
         start = false;
      else
         stream << glue;
      stream << str(f(*first++));
   }
   return stream.str();
}

template<typename InputIt>
string implode(InputIt first, InputIt last, const std::string_view glue)
{
   auto f = [](const decltype(*first)& v) -> std::string { return str(v); };
   return implode(first, last, glue, f);
}

namespace rng
{
   template<typename Range, typename F>
   string implode(const Range& rng, const std::string_view glue, F f)
   {
      return ::perceive::implode(cbegin(rng), cend(rng), glue, f);
   }

   template<typename Range>
   string implode(const Range& rng, const std::string_view glue)
   {
      return ::perceive::implode(cbegin(rng), cend(rng), glue);
   }

} // namespace rng

std::vector<std::string> explode(const std::string_view line,
                                 const std::string_view delims,
                                 const bool collapse_empty_fields
                                 = false) noexcept(false); // std::bad_alloc

// ----------------------------------------------------------------- Begins with

template<class U, class V>
constexpr bool begins_with(const U& input, const V& match)
{
   return input.size() >= match.size()
          and std::equal(cbegin(match), cend(match), begin(input));
}

template<class U, class V>
constexpr bool starts_with(const U& input, const V& match)
{
   return begins_with(input, match);
}

template<class U, class V>
constexpr bool ends_with(const U& input, const V& match)
{
   return input.size() >= match.size()
          and std::equal(crbegin(match), crend(match), rbegin(input));
}

// -------------------------------------------------------- Substitute variables
// Like sub-${VAR} => sub-what-var-was
string substitute_variables(const std::unordered_map<string, string>& vars,
                            const string& line_s);

// ------------------------------------------------- Pretty Printing binary data

string str(const void* data, size_t sz);

// ------------------------------------------------------------------------ Trim

inline void ltrim(std::string& s)
{
   s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
              return !std::isspace(ch);
           }));
}

// trim from end (in place)
inline void rtrim(std::string& s)
{
   s.erase(std::find_if(
               s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); })
               .base(),
           s.end());
}

// trim from both ends (in place)
inline void trim(std::string& s)
{
   ltrim(s);
   rtrim(s);
}

inline string trim_copy(const std::string& s)
{
   auto ret = s;
   trim(ret);
   return ret;
}

// --------------------------------------------------------- synchronized output

inline void sync_write(std::function<void()> thunk)
{
   static std::mutex padlock;
   std::lock_guard<decltype(padlock)> lock(padlock);
   // This thunk should do the writing to cout, or whatever nees to be
   // synchronised.
   thunk();
}

inline void sync_write(std::ostream& os, const std::string& s)
{
   sync_write([&]() {
      os << s;
      os.flush();
   });
}

// ---------------------------------------------------------------- strip suffix

inline string strip_suffix(const string& s, char delim = '.') noexcept
{
   auto pos = s.find_last_of(delim);
   return (pos == string::npos) ? s : s.substr(0, pos);
}

inline std::pair<string, string> extract_suffix(const string& s,
                                                char delim = '.') noexcept
{
   auto pos = s.find_last_of(delim);
   return (pos == string::npos)
              ? std::pair<string, string>(s, {})
              : std::pair<string, string>(s.substr(0, pos), s.substr(pos));
}

// ------------------------------------------------------------------ string-is?

bool string_is_uppercase(const std::string_view s) noexcept;

string string_to_uppercase(const std::string_view s) noexcept;

// --------------------------------------------------------------- mem-usage-str

string mem_usage_str(size_t bytes) noexcept(false);

// ------------------------------------------------------------ encode-dot-label
// don't care about bad_alloc
string encode_dot_label(const std::string_view s) noexcept;

} // namespace perceive
