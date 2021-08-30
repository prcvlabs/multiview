
#pragma once

#include <functional>

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
void render_tiny(const string_view s, std::function<void(int x, int y)> f);
void render_tiny_mask(const string_view s, std::function<void(int x, int y)> f);
void render_tiny_dimensions(const string_view s, int& width, int& height);
inline Point2 render_tiny_dimensions(const string_view s)
{
   Point2 xy;
   render_tiny_dimensions(s, xy.x, xy.y);
   return xy;
}

void render_tiny_8x8(const string_view s, std::function<void(int x, int y)> f);
void render_tiny_mask_8x8(const string_view s,
                          std::function<void(int x, int y)> f);
void render_tiny_dimensions_8x8(const string_view s, int& width, int& height);

inline void render_string_f(const string_view& s,
                            const uint32_t k,
                            const uint32_t background_k,
                            std::function<void(int, int, uint32_t)> f)
{
   int w = 0, h = 0;
   render_tiny_dimensions(s, w, h);
   render_tiny_mask(s.data(), [&](int x, int y) { f(x, y, background_k); });
   render_tiny(s.data(), [&](int x, int y) { f(x, y, k); });
}

template<typename T>
void render_string(T& im,
                   const string_view s,
                   const Point2& p,
                   const uint32_t foreground_k,
                   const uint32_t background_k)
{
   render_string_f(
       s, foreground_k, background_k, [&](auto x0, auto y0, auto k) {
          int x = x0 + p.x;
          int y = y0 + p.y;
          set(im, x, y, k);
       });
}

template<typename T>
void render_string_8x8(T& im,
                       const string& s,
                       const Point2& p,
                       const uint32_t k,
                       const uint32_t background_k)
{
   int w = 0, h = 0;
   render_tiny_dimensions_8x8(s.c_str(), w, h);

   // Find the image point from the 'x', 'y' offset
   auto pos = [&](int x, int y) { return Point2(x + p.x, y + p.y - h); };

   render_tiny_8x8(s.c_str(), [&](int x, int y) {
      if(im.in_bounds(pos(x, y))) im(pos(x, y)) = k;
   });
   render_tiny_mask_8x8(s.c_str(), [&](int x, int y) {
      if(im.in_bounds(pos(x, y))) im(pos(x, y)) = background_k;
   });
}

} // namespace perceive
