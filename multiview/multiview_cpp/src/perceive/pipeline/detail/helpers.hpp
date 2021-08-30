
#pragma once

#include "stdinc.hpp"

namespace perceive
{
inline void println(std::string s)
{
   sync_write(cout, format("{}{}\n", s, ANSI_COLOUR_RESET));
}

inline void simple_banner(std::string s, const char* colour = nullptr)
{
   const unsigned w = 80;
   unsigned off     = (w - unsigned(s.size()));
   std::string padding(off, ' ');
   if(colour == nullptr)
      println(s);
   else
      println(format("{}{}{}", colour, s, ANSI_COLOUR_RESET));
}

inline void print_timing(const string& s)
{
   const char* process_timer_colour = ANSI_COLOUR_CYAN;
   println(format("{} \u2739 {}{}{}",
                  ANSI_COLOUR_YELLOW,
                  ANSI_COLOUR_RESET,
                  ANSI_COLOUR_CYAN,
                  s));
}

} // namespace perceive
