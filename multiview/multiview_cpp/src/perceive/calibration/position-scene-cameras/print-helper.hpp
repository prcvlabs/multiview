
#pragma once

namespace perceive::calibration::position_scene_cameras
{
inline void print(const string_view glyph,
                  const string_view color,
                  const string_view msg,
                  const bool print_newline = true)
{
   cout << format(" {}  {}{}\x1b[0m", glyph, color, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

constexpr auto g_info         = "\x1b[37m\u261b\x1b[0m";
constexpr auto g_skull        = "\x1b[91m\u2620\x1b[0m";
constexpr auto g_radioactive  = "\x1b[91m\u2622\x1b[0m";
constexpr auto g_dotted_sq    = "\x1b[96m\u2b1a\x1b[0m";
constexpr auto g_bullet       = "\x1b[0m\u2738\x1b[0m";
constexpr auto g_cross_arrows = "\x1b[96m\u2928\x1b[0m";
constexpr auto g_waves        = "\x1b[96m\u29da\x1b[0m";
constexpr auto g_wave         = "\x1b[96m\u223c\x1b[0m";
constexpr auto g_wedge        = "\x1b[0m\u2023\x1b[0m";
constexpr auto g_cross        = "\x1b[96m\u2613\x1b[0m";
constexpr auto g_victory      = "\x1b[40m\x1b[97m\u270c\x1b[0m";
constexpr auto g_coffee       = "\x1b[40m\x1b[97m\u26fe\x1b[0m";
constexpr auto g_tick         = "\x1b[40m\x1b[92m\u2714\x1b[0m";

constexpr auto g_default    = "\x1b[0m";
constexpr auto g_red        = "\x1b[31m";
constexpr auto g_error      = "\x1b[4m\x1b[91m";
constexpr auto g_light_gray = "\x1b[37m";
constexpr auto g_light_red  = "\x1b[91m";
constexpr auto g_white      = "\x1b[97m";

} // namespace perceive::calibration::position_scene_cameras
