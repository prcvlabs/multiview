
#pragma once

#include "fmt/format.h"
#include "string-utils.hpp"
#include <mutex>

#ifdef DEBUG_BUILD
#define DLOG(m) \
   ::perceive::Logger::report(0, __FILE__, __LINE__, ::perceive::str(m))
#else
#define DLOG(m)
#endif

#define INFO(m)                         \
   if(::perceive::get_log_level() <= 1) \
   ::perceive::Logger::report(1, __FILE__, __LINE__, ::perceive::str(m))
#define WARN(m)                         \
   if(::perceive::get_log_level() <= 2) \
   ::perceive::Logger::report(2, __FILE__, __LINE__, ::perceive::str(m))
#define LOG_ERR(m)                      \
   if(::perceive::get_log_level() <= 3) \
   ::perceive::Logger::report(3, __FILE__, __LINE__, ::perceive::str(m))
#define FATAL(m) \
   ::perceive::Logger::report(4, __FILE__, __LINE__, ::perceive::str(m))
#define TRACE(m)                          \
   if(::perceive::multiview_trace_mode()) \
   ::perceive::Logger::report(5, __FILE__, __LINE__, ::perceive::str(m))

#define STRONG_ASSERT(expr, m) \
   {                           \
      if(!(expr)) FATAL(m);    \
   }

namespace perceive
{
using fmt::format;
using std::string;

inline void set_logger_debug_out_file(string filename = "");
inline void set_logger_info_out_file(string filename = "");
inline void set_logger_warn_out_file(string filename = "");
inline void set_logger_error_out_file(string filename = "");
inline void set_logger_fatal_out_file(string filename = "");

inline void logger_enable_colours(bool value);
inline bool logger_colours_endabled();

inline int get_log_level();
inline void set_log_info();  // 1
inline void set_log_warn();  // 2
inline void set_log_error(); // 3
// Fatal is always logged

inline void set_log_flag(bool on);
inline bool get_log_flag();

class Logger;

// For **documentation** see implementation of this function:
// impl/impl_logging.hpp
inline void logging_example();

} // namespace perceive

// -------------------------------------------------------------- Implementation

namespace perceive
{
class Logger
{
 private:
   static Logger* instance(bool delete_instance = false)
   {
      static Logger instance_; // This is now thread-safe in C++11
      return &instance_;
   }

   static const char* level_to_string(int level)
   {
      if(colours_enabled()) {
         switch(level) {
         case 0: return ANSI_COLOUR_CYAN "DEBUG" ANSI_COLOUR_RESET;
         case 1: return ANSI_COLOUR_BLUE "INFO " ANSI_COLOUR_RESET;
         case 2: return ANSI_COLOUR_YELLOW "WARN " ANSI_COLOUR_RESET;
         case 3: return ANSI_COLOUR_RED "ERROR" ANSI_COLOUR_RESET;
         case 4: return ANSI_COLOUR_RED "FATAL" ANSI_COLOUR_RESET;
         case 5:
            return "\x1b[42m\x1b[97m"
                   "TRACE" ANSI_COLOUR_RESET;
         default: assert(false); break;
         }
      } else {
         switch(level) {
         case 0: return "DEBUG";
         case 1: return "INFO ";
         case 2: return "WARN ";
         case 3: return "ERROR";
         case 4: return "FATAL";
         case 5: return "TRACE";
         default: assert(false); break;
         }
      }
      return "?";
   }

   bool _colours;
   int _log_level;
   bool _flag;

   Logger() { init(); }
   ~Logger() = default;

   void init()
   {
#ifdef MSVCPP
      _colours = false;
#else
      _colours = true;
#endif
      _flag      = false;
      _log_level = 0;
   }

 public:
   static void set_flag(bool on) { instance()->_flag = on; }
   static bool get_flag() { return instance()->_flag; }

   static void
   report(int level, const char* file, int lineno, const string& msg)
   {
      // TODO, this must be non-blocking
      if((level >= log_level() && level <= 5) || level == 0) {
         std::ostream& out = std::cout;
         sync_write([&]() {
            out << level_to_string(level) << " " << ANSI_COLOUR_GREY << file
                << ":" << lineno << ANSI_COLOUR_RESET << " " << msg << "\n";
         });
         if(level == 4) {
            exit(1); // Die on fatal
         }
      }
   }

   static int log_level() { return instance()->_log_level; }

   static void set_log_level(int level)
   {
      if(level > 0 && level <= 5)
         instance()->_log_level = level;
      else
         report(
             2, __FILE__, __LINE__, format("Invalid log level: {}", level));
   }

   static bool colours_enabled() { return instance()->_colours; }

   static void enable_colours(bool value) { instance()->_colours = value; }
};

template<class T> constexpr std::string_view type_name()
{
   using namespace std;
#ifdef __clang__
   string_view p = __PRETTY_FUNCTION__;
   return string_view(p.data() + 34, p.size() - 34 - 1);
#elif defined(__GNUC__)
   string_view p = __PRETTY_FUNCTION__;
#if __cplusplus < 201402
   return string_view(p.data() + 36, p.size() - 36 - 1);
#else
   return string_view(p.data() + 49, p.find(';', 49) - 49);
#endif
#elif defined(_MSC_VER)
   string_view p = __FUNCSIG__;
   return string_view(p.data() + 84, p.size() - 84 - 7);
#endif
}

} // namespace perceive

inline int ::perceive::get_log_level()
{
   return ::perceive::Logger::log_level();
}
inline void ::perceive::set_log_info() { ::perceive::Logger::set_log_level(1); }
inline void ::perceive::set_log_warn() { ::perceive::Logger::set_log_level(2); }
inline void ::perceive::set_log_error()
{
   ::perceive::Logger::set_log_level(3);
}

inline void ::perceive::set_log_flag(bool on)
{
   ::perceive::Logger::set_flag(on);
}
inline bool ::perceive::get_log_flag()
{
   return ::perceive::Logger::get_flag();
}

inline void ::perceive::logger_enable_colours(bool value)
{
   ::perceive::Logger::enable_colours(value);
}
inline bool ::perceive::logger_colours_endabled()
{
   return ::perceive::Logger::colours_enabled();
}

namespace perceive
{
// -------------------------------------------- Documentation
inline void logging_example()
{
   // Defaults to cout
   DLOG("Some message");
   INFO(format("some info {}", 3));

   // Defaults to cerr
   WARN(format("Do not like: '{}'", "rain"));
   LOG_ERR("Some error message");

   // Redirect outputs to file.
   // (If file cannot be opened, then an error is logged,
   // and output continues as before
   WARN(format("This warning being logged to {} file.", 5)); //
   DLOG(4); // Outputs DEBUG __FILE__:__LINE__ 4 (prtty cool, eh =0)

   // Redirect output to default (i.e., cerr)
   LOG_ERR("Okay, another error");

   // FATAL logs an error and calls exit(1)
   FATAL("Okay, hitting the kill-switch");
}

} // namespace perceive
