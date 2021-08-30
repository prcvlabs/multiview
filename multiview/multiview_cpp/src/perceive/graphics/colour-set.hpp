
#pragma once

#include "LAB.hpp"
#include "heat-maps.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/utils/math.hpp"

#include <opencv2/core.hpp>

namespace perceive
{
// @see https://en.wikipedia.org/wiki/Web_colors , X11 color names

// Pink colors
constexpr uint32_t k_pink              = 0xFFC0CB;
constexpr uint32_t k_light_pink        = 0xFFB6C1;
constexpr uint32_t k_hot_pink          = 0xFF69B4;
constexpr uint32_t k_deep_pink         = 0xFF1493;
constexpr uint32_t k_pale_violet_red   = 0xDB7093;
constexpr uint32_t k_medium_violet_red = 0xC71585;

// Red colors
constexpr uint32_t k_light_salmon = 0xFFA07A;
constexpr uint32_t k_salmon       = 0xFA8072;
constexpr uint32_t k_dark_salmon  = 0xE9967A;
constexpr uint32_t k_light_coral  = 0xF08080;
constexpr uint32_t k_indian_red   = 0xCD5C5C;
constexpr uint32_t k_crimson      = 0xDC143C;
constexpr uint32_t k_fire_brick   = 0xB22222;
constexpr uint32_t k_dark_red     = 0x8B0000;
constexpr uint32_t k_red          = 0xFF0000;

// Orange colors
constexpr uint32_t k_orange_red             = 0xFF4500;
constexpr uint32_t k_tomato                 = 0xFF6347;
constexpr uint32_t k_coral                  = 0xFF7F50;
constexpr uint32_t k_dark_orange            = 0xFF8C00;
constexpr uint32_t k_orange                 = 0xFFA500;
constexpr uint32_t k_yellow                 = 0xFFFF00;
constexpr uint32_t k_light_yellow           = 0xFFFFE0;
constexpr uint32_t k_lemon_chiffon          = 0xFFFACD;
constexpr uint32_t k_light_goldenrod_yellow = 0xFAFAD2;
constexpr uint32_t k_papaya_whip            = 0xFFEFD5;
constexpr uint32_t k_moccasin               = 0xFFE4B5;
constexpr uint32_t k_peach_puff             = 0xFFDAB9;
constexpr uint32_t k_pale_goldenrod         = 0xEEE8AA;
constexpr uint32_t k_khaki                  = 0xF0E68C;
constexpr uint32_t k_dark_khaki             = 0xBDB76B;
constexpr uint32_t k_gold                   = 0xFFD700;

// Brown colors
constexpr uint32_t k_cornsilk        = 0xFFF8DC;
constexpr uint32_t k_blanched_almond = 0xFFEBCD;
constexpr uint32_t k_bisque          = 0xFFE4C4;
constexpr uint32_t k_navajo_white    = 0xFFDEAD;
constexpr uint32_t k_wheat           = 0xF5DEB3;
constexpr uint32_t k_burly_wood      = 0xDEB887;
constexpr uint32_t k_tan             = 0xD2B48C;
constexpr uint32_t k_rosy_brown      = 0xBC8F8F;
constexpr uint32_t k_sandy_brown     = 0xF4A460;
constexpr uint32_t k_goldenrod       = 0xDAA520;
constexpr uint32_t k_dark_goldenrod  = 0xB8860B;
constexpr uint32_t k_peru            = 0xCD853F;
constexpr uint32_t k_chocolate       = 0xD2691E;
constexpr uint32_t k_saddle_brown    = 0x8B4513;
constexpr uint32_t k_sienna          = 0xA0522D;
constexpr uint32_t k_brown           = 0xA52A2A;
constexpr uint32_t k_maroon          = 0x800000;

// Green colors
constexpr uint32_t k_dark_olive_green    = 0x556B2F;
constexpr uint32_t k_olive               = 0x808000;
constexpr uint32_t k_olive_drab          = 0x6B8E23;
constexpr uint32_t k_yellow_green        = 0x9ACD32;
constexpr uint32_t k_lime_green          = 0x32CD32;
constexpr uint32_t k_lime                = 0x00FF00;
constexpr uint32_t k_lawn_green          = 0x7CFC00;
constexpr uint32_t k_chartreuse          = 0x7FFF00;
constexpr uint32_t k_green_yellow        = 0xADFF2F;
constexpr uint32_t k_spring_green        = 0x00FF7F;
constexpr uint32_t k_medium_spring_green = 0x00FA9A;
constexpr uint32_t k_light_green         = 0x90EE90;
constexpr uint32_t k_pale_green          = 0x98FB98;
constexpr uint32_t k_dark_sea_green      = 0x8FBC8F;
constexpr uint32_t k_medium_aquamarine   = 0x66CDAA;
constexpr uint32_t k_medium_sea_green    = 0x3CB371;
constexpr uint32_t k_sea_green           = 0x2E8B57;
constexpr uint32_t k_forest_green        = 0x228B22;
constexpr uint32_t k_green               = 0x008000;
constexpr uint32_t k_dark_green          = 0x006400;

// Cyan colors
constexpr uint32_t k_aqua             = 0x00FFFF;
constexpr uint32_t k_cyan             = 0x00FFFF;
constexpr uint32_t k_light_cyan       = 0xE0FFFF;
constexpr uint32_t k_pale_turquoise   = 0xAFEEEE;
constexpr uint32_t k_aquamarine       = 0x7FFFD4;
constexpr uint32_t k_turquoise        = 0x40E0D0;
constexpr uint32_t k_medium_turquoise = 0x48D1CC;
constexpr uint32_t k_dark_turquoise   = 0x00CED1;
constexpr uint32_t k_light_sea_green  = 0x20B2AA;
constexpr uint32_t k_cadet_blue       = 0x5F9EA0;
constexpr uint32_t k_dark_cyan        = 0x008B8B;
constexpr uint32_t k_teal             = 0x008080;

// Blue colors
constexpr uint32_t k_light_steel_blue = 0xB0C4DE;
constexpr uint32_t k_powder_blue      = 0xB0E0E6;
constexpr uint32_t k_light_blue       = 0xADD8E6;
constexpr uint32_t k_sky_blue         = 0x87CEEB;
constexpr uint32_t k_light_sky_blue   = 0x87CEFA;
constexpr uint32_t k_deep_sky_blue    = 0x00BFFF;
constexpr uint32_t k_dodger_blue      = 0x1E90FF;
constexpr uint32_t k_cornflower_blue  = 0x6495ED;
constexpr uint32_t k_steel_blue       = 0x4682B4;
constexpr uint32_t k_royal_blue       = 0x4169E1;
constexpr uint32_t k_blue             = 0x0000FF;
constexpr uint32_t k_medium_blue      = 0x0000CD;
constexpr uint32_t k_dark_blue        = 0x00008B;
constexpr uint32_t k_navy             = 0x000080;
constexpr uint32_t k_midnight_blue    = 0x191970;

// Purple, violet, and magenta colors
constexpr uint32_t k_lavender          = 0xE6E6FA;
constexpr uint32_t k_thistle           = 0xD8BFD8;
constexpr uint32_t k_plum              = 0xDDA0DD;
constexpr uint32_t k_violet            = 0xEE82EE;
constexpr uint32_t k_orchid            = 0xDA70D6;
constexpr uint32_t k_fuchsia           = 0xFF00FF;
constexpr uint32_t k_magenta           = 0xFF00FF;
constexpr uint32_t k_medium_orchid     = 0xBA55D3;
constexpr uint32_t k_medium_purple     = 0x9370DB;
constexpr uint32_t k_blue_violet       = 0x8A2BE2;
constexpr uint32_t k_dark_violet       = 0x9400D3;
constexpr uint32_t k_dark_orchid       = 0x9932CC;
constexpr uint32_t k_dark_magenta      = 0x8B008B;
constexpr uint32_t k_purple            = 0x800080;
constexpr uint32_t k_indigo            = 0x4B0082;
constexpr uint32_t k_dark_slate_blue   = 0x483D8B;
constexpr uint32_t k_slate_blue        = 0x6A5ACD;
constexpr uint32_t k_medium_slate_blue = 0x7B68EE;

// White colors
constexpr uint32_t k_white          = 0xFFFFFF;
constexpr uint32_t k_snow           = 0xFFFAFA;
constexpr uint32_t k_honeydew       = 0xF0FFF0;
constexpr uint32_t k_mint_cream     = 0xF5FFFA;
constexpr uint32_t k_azure          = 0xF0FFFF;
constexpr uint32_t k_alice_blue     = 0xF0F8FF;
constexpr uint32_t k_ghost_white    = 0xF8F8FF;
constexpr uint32_t k_white_smoke    = 0xF5F5F5;
constexpr uint32_t k_seashell       = 0xFFF5EE;
constexpr uint32_t k_beige          = 0xF5F5DC;
constexpr uint32_t k_old_lace       = 0xFDF5E6;
constexpr uint32_t k_floral_white   = 0xFFFAF0;
constexpr uint32_t k_ivory          = 0xFFFFF0;
constexpr uint32_t k_antique_white  = 0xFAEBD7;
constexpr uint32_t k_linen          = 0xFAF0E6;
constexpr uint32_t k_lavender_blush = 0xFFF0F5;
constexpr uint32_t k_misty_rose     = 0xFFE4E1;

// Gray and black colors
constexpr uint32_t k_gainsboro        = 0xDCDCDC;
constexpr uint32_t k_light_gray       = 0xD3D3D3;
constexpr uint32_t k_silver           = 0xC0C0C0;
constexpr uint32_t k_dark_gray        = 0xA9A9A9;
constexpr uint32_t k_gray             = 0x808080;
constexpr uint32_t k_dim_gray         = 0x696969;
constexpr uint32_t k_light_slate_gray = 0x778899;
constexpr uint32_t k_slate_gray       = 0x708090;
constexpr uint32_t k_dark_slate_gray  = 0x2F4F4F;
constexpr uint32_t k_black            = 0x000000;

const uint32_t k_crayons[] = {k_navy,
                              k_papaya_whip,
                              k_seashell,
                              k_dim_gray,
                              k_linen,
                              k_cadet_blue,
                              k_ivory,
                              k_blanched_almond,
                              k_hot_pink,
                              k_navajo_white,
                              k_lavender,
                              k_tan,
                              k_forest_green,
                              k_burly_wood,
                              k_cyan,
                              k_dark_gray,
                              k_lavender_blush,
                              k_olive,
                              k_steel_blue,
                              k_slate_gray,
                              k_dark_olive_green,
                              k_blue,
                              k_slate_blue,
                              k_pale_green,
                              k_medium_aquamarine,
                              k_floral_white,
                              k_medium_purple,
                              k_gainsboro,
                              k_pale_goldenrod,
                              k_dark_cyan,
                              k_misty_rose,
                              k_dark_orange,
                              k_lawn_green,
                              k_cornsilk,
                              k_ghost_white,
                              k_powder_blue,
                              k_light_salmon,
                              k_lemon_chiffon,
                              k_goldenrod,
                              k_dark_sea_green,
                              k_medium_turquoise,
                              k_dark_salmon,
                              k_orchid,
                              k_dark_violet,
                              k_dark_red,
                              k_brown,
                              k_dark_slate_blue,
                              k_dark_blue,
                              k_tomato,
                              k_thistle,
                              k_light_cyan,
                              k_dark_turquoise,
                              k_light_pink,
                              k_rosy_brown,
                              k_fire_brick,
                              k_wheat,
                              k_violet,
                              k_light_yellow,
                              k_azure,
                              k_peach_puff,
                              k_khaki,
                              k_indigo,
                              k_dark_magenta,
                              k_dark_goldenrod,
                              k_coral,
                              k_light_goldenrod_yellow,
                              k_midnight_blue,
                              k_turquoise,
                              k_dark_orchid,
                              k_teal,
                              k_orange,
                              k_indian_red,
                              k_spring_green,
                              k_aquamarine,
                              k_sea_green,
                              k_sienna,
                              k_medium_sea_green,
                              k_sandy_brown,
                              k_white_smoke,
                              k_blue_violet,
                              k_light_blue,
                              k_purple,
                              k_medium_orchid,
                              k_old_lace,
                              k_olive_drab,
                              k_salmon,
                              k_light_gray,
                              k_snow,
                              k_light_sky_blue,
                              k_deep_sky_blue,
                              k_dark_khaki,
                              k_honeydew,
                              k_medium_violet_red,
                              k_medium_spring_green,
                              k_deep_pink,
                              k_red,
                              k_cornflower_blue,
                              k_pale_violet_red,
                              k_sky_blue,
                              k_moccasin,
                              k_medium_blue,
                              k_chocolate,
                              k_saddle_brown,
                              k_dodger_blue,
                              k_aqua,
                              k_plum,
                              k_lime_green,
                              k_light_green,
                              k_light_sea_green,
                              k_dark_green,
                              k_gold,
                              k_crimson,
                              k_green,
                              k_light_slate_gray,
                              k_antique_white,
                              k_gray,
                              k_light_steel_blue,
                              k_maroon,
                              k_beige,
                              k_peru,
                              k_bisque,
                              k_medium_slate_blue,
                              k_orange_red,
                              k_magenta,
                              k_fuchsia,
                              k_alice_blue,
                              k_silver,
                              k_yellow_green,
                              k_yellow,
                              k_pink,
                              k_green_yellow,
                              k_royal_blue,
                              k_pale_turquoise,
                              k_chartreuse,
                              k_light_coral,
                              k_dark_slate_gray,
                              k_mint_cream,
                              k_lime};

inline unsigned n_crayons() noexcept
{
   return sizeof(k_crayons) / sizeof(uint32_t);
}

inline uint32_t colour_set_1(unsigned i) noexcept
{
   return k_crayons[modulo(i, n_crayons())];
}

const uint32_t k_sorted_crayons[] = {k_pink,
                                     k_light_pink,
                                     k_hot_pink,
                                     k_deep_pink,
                                     k_pale_violet_red,
                                     k_medium_violet_red,
                                     k_light_salmon,
                                     k_salmon,
                                     k_dark_salmon,
                                     k_light_coral,
                                     k_indian_red,
                                     k_crimson,
                                     k_fire_brick,
                                     k_dark_red,
                                     k_red,
                                     k_orange_red,
                                     k_tomato,
                                     k_coral,
                                     k_dark_orange,
                                     k_orange,
                                     k_yellow,
                                     k_light_yellow,
                                     k_lemon_chiffon,
                                     k_light_goldenrod_yellow,
                                     k_papaya_whip,
                                     k_moccasin,
                                     k_peach_puff,
                                     k_pale_goldenrod,
                                     k_khaki,
                                     k_dark_khaki,
                                     k_gold,
                                     k_cornsilk,
                                     k_blanched_almond,
                                     k_bisque,
                                     k_navajo_white,
                                     k_wheat,
                                     k_burly_wood,
                                     k_tan,
                                     k_rosy_brown,
                                     k_sandy_brown,
                                     k_goldenrod,
                                     k_dark_goldenrod,
                                     k_peru,
                                     k_chocolate,
                                     k_saddle_brown,
                                     k_sienna,
                                     k_brown,
                                     k_maroon,
                                     k_dark_olive_green,
                                     k_olive,
                                     k_olive_drab,
                                     k_yellow_green,
                                     k_lime_green,
                                     k_lime,
                                     k_lawn_green,
                                     k_chartreuse,
                                     k_green_yellow,
                                     k_spring_green,
                                     k_medium_spring_green,
                                     k_light_green,
                                     k_pale_green,
                                     k_dark_sea_green,
                                     k_medium_aquamarine,
                                     k_medium_sea_green,
                                     k_sea_green,
                                     k_forest_green,
                                     k_green,
                                     k_dark_green,
                                     k_aqua,
                                     k_cyan,
                                     k_light_cyan,
                                     k_pale_turquoise,
                                     k_aquamarine,
                                     k_turquoise,
                                     k_medium_turquoise,
                                     k_dark_turquoise,
                                     k_light_sea_green,
                                     k_cadet_blue,
                                     k_dark_cyan,
                                     k_teal,
                                     k_light_steel_blue,
                                     k_powder_blue,
                                     k_light_blue,
                                     k_sky_blue,
                                     k_light_sky_blue,
                                     k_deep_sky_blue,
                                     k_dodger_blue,
                                     k_cornflower_blue,
                                     k_steel_blue,
                                     k_royal_blue,
                                     k_blue,
                                     k_medium_blue,
                                     k_dark_blue,
                                     k_navy,
                                     k_midnight_blue,
                                     k_lavender,
                                     k_thistle,
                                     k_plum,
                                     k_violet,
                                     k_orchid,
                                     k_fuchsia,
                                     k_magenta,
                                     k_medium_orchid,
                                     k_medium_purple,
                                     k_blue_violet,
                                     k_dark_violet,
                                     k_dark_orchid,
                                     k_dark_magenta,
                                     k_purple,
                                     k_indigo,
                                     k_dark_slate_blue,
                                     k_slate_blue,
                                     k_medium_slate_blue,
                                     k_white,
                                     k_snow,
                                     k_honeydew,
                                     k_mint_cream,
                                     k_azure,
                                     k_alice_blue,
                                     k_ghost_white,
                                     k_white_smoke,
                                     k_seashell,
                                     k_beige,
                                     k_old_lace,
                                     k_floral_white,
                                     k_ivory,
                                     k_antique_white,
                                     k_linen,
                                     k_lavender_blush,
                                     k_misty_rose,
                                     k_gainsboro,
                                     k_light_gray,
                                     k_silver,
                                     k_dark_gray,
                                     k_gray,
                                     k_dim_gray,
                                     k_light_slate_gray,
                                     k_slate_gray,
                                     k_dark_slate_gray,
                                     k_black};

inline unsigned n_sorted_crayons() noexcept
{
   return sizeof(k_sorted_crayons) / sizeof(uint32_t);
}

const uint32_t k_colour_set_2[] = {k_sandy_brown,
                                   k_medium_blue,
                                   k_chocolate,
                                   k_chartreuse,
                                   k_dark_green,
                                   k_light_goldenrod_yellow,
                                   k_teal,
                                   k_black,
                                   k_wheat,
                                   k_cornsilk,
                                   k_light_gray,
                                   k_light_green,
                                   k_orchid,
                                   k_pink,
                                   k_slate_blue,
                                   k_orange_red,
                                   k_cornflower_blue,
                                   k_pale_goldenrod,
                                   k_linen,
                                   k_dark_turquoise,
                                   k_ghost_white,
                                   k_salmon,
                                   k_maroon,
                                   k_medium_sea_green,
                                   k_medium_purple,
                                   k_dark_magenta,
                                   k_orange,
                                   k_deep_pink,
                                   k_crimson,
                                   k_sky_blue,
                                   k_pale_turquoise,
                                   k_yellow_green,
                                   k_lavender,
                                   k_dim_gray};

inline unsigned n_colour_set_2() noexcept
{
   return sizeof(k_colour_set_2) / sizeof(uint32_t);
}

inline uint32_t colour_set_2(unsigned i) noexcept
{
   return k_colour_set_2[modulo(i, n_colour_set_2())];
}

const uint32_t k_colour_set_3[] = {k_red,
                                   k_green,
                                   k_yellow,
                                   k_dark_violet,
                                   k_tan,
                                   k_sky_blue,
                                   k_indigo,
                                   k_teal};

inline unsigned n_colour_set_3() noexcept
{
   return sizeof(k_colour_set_3) / sizeof(uint32_t);
}

inline uint32_t colour_set_3(unsigned i) noexcept
{
   return k_colour_set_3[modulo(i, n_colour_set_3())];
}

const uint32_t k_colour_set_4[] = {k_crimson,
                                   k_green,
                                   k_yellow,
                                   k_dark_violet,
                                   k_orange,
                                   k_sky_blue,
                                   k_indigo,
                                   k_teal,
                                   k_dark_green,
                                   k_pink,
                                   k_yellow_green,
                                   k_dim_gray,
                                   k_beige,
                                   k_midnight_blue,
                                   k_light_cyan};

inline constexpr unsigned n_colour_set_4() noexcept
{
   return sizeof(k_colour_set_4) / sizeof(uint32_t);
}

template<std::integral I> inline constexpr uint32_t colour_set_4(I i) noexcept
{
   return k_colour_set_4[modulo(unsigned(i), n_colour_set_4())];
}

constexpr uint32_t k_colour_set_5[]
    = {k_crimson, k_spring_green, k_yellow, k_orange, k_sky_blue};

inline constexpr unsigned n_colour_set_5() noexcept
{
   return sizeof(k_colour_set_5) / sizeof(uint32_t);
}

inline constexpr uint32_t colour_set_5(unsigned i) noexcept
{
   return k_colour_set_5[modulo(i, n_colour_set_5())];
}

// ------------------------------------------------------------- "Member" access
//
inline constexpr uint8_t alpha(uint32_t k) noexcept
{
   return (uint32_t(k) & 0xff000000u) >> 24;
}
inline constexpr uint8_t red(uint32_t k) noexcept
{
   return (uint32_t(k) & 0x00ff0000u) >> 16;
}
inline constexpr uint8_t green(uint32_t k) noexcept
{
   return (uint32_t(k) & 0x0000ff00u) >> 8;
}
inline constexpr uint8_t blue(uint32_t k) noexcept
{
   return (uint32_t(k) & 0x000000ffu) >> 0;
}

// ------------------------------------------------------ Vector3 to/from uint32
inline Vector3 kolour_to_vector3(uint32_t k) noexcept
{
   return Vector3(((k >> 16) & 0xff) / 255.0,
                  ((k >> 8) & 0xff) / 255.0,
                  ((k >> 0) & 0xff) / 255.0);
}

inline Vector4f kolour_to_vector4f(uint32_t k, float alpha) noexcept
{
   return Vector4f(((k >> 16) & 0xff) / 255.0f,
                   ((k >> 8) & 0xff) / 255.0f,
                   ((k >> 0) & 0xff) / 255.0f,
                   alpha);
}

inline constexpr uint32_t vector3_to_kolour(const Vector3& k) noexcept
{
   uint8_t r = uint8_t(k.x * 255.0);
   uint8_t g = uint8_t(k.y * 255.0);
   uint8_t b = uint8_t(k.z * 255.0);
   return (uint32_t(r) << 16) | (uint32_t(g) << 8) | (uint32_t(b) << 0);
}

template<std::integral I>
inline constexpr uint32_t make_colour(I r, I g, I b) noexcept
{
   return ((uint32_t(r) & 0xff) << 16) | ((uint32_t(g) & 0xff) << 8)
          | ((uint32_t(b) & 0xff) << 0);
}

inline constexpr uint32_t
make_colour(uint8_t a, uint8_t r, uint8_t g, uint8_t b) noexcept
{
   return (uint32_t(a) << 24) | (uint32_t(r) << 16) | (uint32_t(g) << 8)
          | (uint32_t(b) << 0);
}

// ------------------------------------------------------------------------- HSV

Vector3 rgb_to_hsv(const Vector3& in) noexcept;
Vector3 hsv_to_rgb(const Vector3& in) noexcept;
inline uint32_t rgb_to_hsv(uint32_t in) noexcept
{
   return vector3_to_kolour(rgb_to_hsv(kolour_to_vector3(in)));
}

inline uint32_t hsv_to_rgb_uint32(real h, real s, real v) noexcept
{
   return vector3_to_kolour(hsv_to_rgb(Vector3(h, s, v)));
}

inline Vector3 kolour_to_hsv(const uint32_t in) noexcept
{
   return rgb_to_hsv(kolour_to_vector3(in));
}

// ------------------------------------------------------------------- cv::Vec3b
//
inline uint32_t vec3b_to_rgb(const cv::Vec3b& X) noexcept
{
   return make_colour(X[2], X[1], X[0]);
}

inline cv::Vec3b rgb_to_vec3b(const uint32_t X) noexcept
{
   return cv::Vec3b(blue(X), green(X), red(X));
}

// ------------------------------------------------------------------------- LAB
//
Vector3 rgb_to_lab(const Vector3& in) noexcept;
Vector3 rgb_to_lab(uint32_t in) noexcept;

Vector3 lab_to_rgb(const Vector3& in) noexcept;

inline uint32_t lab_to_kolour(const Vector3& in) noexcept
{
   return vector3_to_kolour(lab_to_rgb(in));
}

inline uint32_t lab_to_kolour(const Vector3f& in) noexcept
{
   return vector3_to_kolour(lab_to_rgb(to_vec3(in)));
}

// These are the expected mean and stddev for compare functions
// on random pairs of crayons (above)
// cie1976, mean = 76.425318, stddev = 39.138304
// cie2000, mean = 40.454945, stddev = 18.571403
inline real cie1976_compare(const Vector3& a, const Vector3& b) noexcept
{
   return (a - b).norm();
}
real cie2000_compare(const Vector3& lab1, const Vector3& lab2) noexcept;
real cie2000_compare(const Vector3f& lab1, const Vector3f& lab2) noexcept;

// This cost function is "normalized" to [0..1] using the
// means and stddev listed above.
real cie1976_normalized(const Vector3& lab1, const Vector3& lab2) noexcept;
float cie1976_normalized(const Vector3f& lab1, const Vector3f& lab2) noexcept;

constexpr real cie1976_mean   = 76.425318;
constexpr real cie1976_stddev = 39.138304;
constexpr real cie2000_mean   = 40.454945;
constexpr real cie2000_stddev = 18.571403;

inline real cie2000_score(const Vector3& lab1, const Vector3& lab2) noexcept
{
   auto z = (cie2000_compare(lab1, lab2) - cie2000_mean) / cie2000_stddev;
   return clamp(phi_function(z) * 3.0, 0.0, 1.0);
}

inline real cie2000_score(const uint32_t& k0, const uint32_t& k1) noexcept
{
   return cie2000_score(rgb_to_lab(k0), rgb_to_lab(k1));
}

inline real cie2000_score(const Vector3f& lab1, const Vector3f& lab2) noexcept
{
   return cie2000_score(to_vec3(lab1), to_vec3(lab2));
}

inline real cie2000_score(const cv::Vec3b& pix1, const cv::Vec3b& pix2) noexcept
{ // TODO, surely this is wrong.
   auto ck1 = Vector3(pix1.val[2], pix1.val[1], pix1.val[0]);
   auto ck2 = Vector3(pix2.val[2], pix2.val[1], pix2.val[0]);
   return cie2000_score(ck1, ck2);
}

inline real cie1976_score(const Vector3& lab1, const Vector3& lab2) noexcept
{
   auto z = (cie1976_compare(lab1, lab2) - cie1976_mean) / cie1976_stddev;
   return clamp(phi_function(z) * 3.0, 0.0, 1.0);
}

inline real cie1976_score(const Vector3f& lab1, const Vector3f& lab2) noexcept
{
   return cie1976_score(to_vec3(lab1), to_vec3(lab2));
}

inline real cie1976_score(const uint32_t& k0, const uint32_t& k1) noexcept
{
   return cie1976_score(rgb_to_lab(k0), rgb_to_lab(k1));
}

namespace detail
{
   void test_LAB_conversions() noexcept;
}

// ------------------------------------------------------------------- Greyscale

inline uint8_t rgb_to_grey(uint32_t k) noexcept
{
   auto g = 0.21 * red(k) + 0.72 * green(k) + 0.07 * blue(k);
   return uint8_t(clamp(uint8_t(g + 0.49), uint8_t(0), uint8_t(255)));
}

inline uint8_t rgb_to_gray(uint32_t k) noexcept { return rgb_to_grey(k); }

inline uint32_t grey_to_rgb(uint8_t g) noexcept
{
   uint32_t k = g;
   return (k << 16) | (k << 8) | (k << 0);
}
inline uint32_t gray_to_rgb(uint8_t g) noexcept { return grey_to_rgb(g); }

// ----------------------------------------------------------------------- Blend

inline uint32_t
blend(uint32_t foreground, uint32_t colour, float fg_alpha) noexcept
{
   const uint32_t f = foreground;
   const uint32_t c = colour;

   uint32_t c1 = uint8_t(((f >> 0) & 0xff) * fg_alpha
                         + ((c >> 0) & 0xff) * (1.0f - fg_alpha));
   uint32_t c2 = uint8_t(((f >> 8) & 0xff) * fg_alpha
                         + ((c >> 8) & 0xff) * (1.0f - fg_alpha));
   uint32_t c3 = uint8_t(((f >> 16) & 0xff) * fg_alpha
                         + ((c >> 16) & 0xff) * (1.0f - fg_alpha));

   return (f & 0xff000000u) | (c3 << 16) | (c2 << 8) | (c1 << 0);
}

inline cv::Vec3b
blend(cv::Vec3b foreground, cv::Vec3b colour, float fg_alpha) noexcept
{
   const uint32_t k
       = blend(vec3b_to_rgb(foreground), vec3b_to_rgb(colour), fg_alpha);
   return rgb_to_vec3b(k);
}

// ---------------------------------------------------------------------- Invert

inline uint32_t invert_colour(const uint32_t k) noexcept
{
   return make_colour(255 - red(k), 255 - green(k), 255 - blue(k));
}

// ------------------------------------------------------------- Angle to Colour

inline uint32_t
angle_to_colour(double angle, double sat = 1.0, double val = 1.0) noexcept
{
   if(angle < 0.0 || angle > M_PI + 1.0 * M_PI / 180.0)
      FATAL(format("Expected 0.0 < {} < pi", angle));
   Vector3 hsv(360.0 * angle / M_PI, sat, val);
   return vector3_to_kolour(hsv_to_rgb(hsv));
}

inline uint32_t normalized_score_to_colour(double score,
                                           double sat = 1.0,
                                           double val = 1.0) noexcept
{
   if(!std::isfinite(score)) return k_black;
   return angle_to_colour(
       std::clamp<double>(score * M_PI, 0.0, M_PI), sat, val);
}

} // namespace perceive
