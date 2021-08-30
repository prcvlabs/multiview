
#pragma once

#include "perceive/foundation.hpp"

namespace perceive
{
void make_integral_image(const uint8_t* data,
                         unsigned w,
                         unsigned h,
                         uint32_t* integral);
void make_integral_image(const uint32_t* data,
                         unsigned w,
                         unsigned h,
                         uint32_t* integral);
void make_integral_image(const float* data,
                         unsigned w,
                         unsigned h,
                         float* integral);

template<typename T>
auto sum_region(const T* data, int w, int h, int l, int t, int r, int b)
{
   // NOTE: boundaries are inclusive
   T sum = 0;
   for(int y = t; y <= b; ++y)
      for(int x = l; x <= r; ++x)
         if(x >= 0 && x < int(w) && y >= 0 && y < int(h)) {
            sum += data[x + y * w];
         }
   return sum;
}

template<typename T>
T integral_region(const T* data,
                  const int w,
                  const int h,
                  const int left,
                  const int top,
                  const int right,
                  const int bottom)
{
   // NOTE: boundaries are inclusive
   if(w <= 0 or h <= 0) return T(0);

   const auto l = (left < 0) ? 0 : left;
   const auto t = (top < 0) ? 0 : top;
   const auto r = (right >= w) ? (w - 1) : right;
   const auto b = (bottom >= h) ? (h - 1) : bottom;

   auto M = [&](int x, int y) -> real {
      return (x < 0 or y < 0 or x >= w or y >= h) ? 0.0 : real(data[x + y * w]);
   };

   const auto tl = M(l - 1, t - 1);
   const auto tr = M(r, t - 1);
   const auto bl = M(l - 1, b);
   const auto br = M(r, b);

   assert(br + tl >= tr + bl);

   return T((br + tl) - (tr + bl));
}

inline uint32_t integral_region(const uint32_t* integral_data,
                                unsigned w,
                                unsigned h,
                                int left,
                                int top,
                                int right,
                                int bottom)
{
   // NOTE: boundaries are inclusive
   if(w == 0 || h == 0) return 0;
   int ih = int(h), iw = int(w);
   if(right >= int(w)) right = int(w) - 1;
   if(bottom >= int(h)) bottom = int(h) - 1;
   if(right < left || bottom < top) return 0;

   auto M = [&](int x, int y) -> uint32_t {
      uint32_t ret = (unsigned(x) < w && unsigned(y) < h)
                         ? integral_data[size_t(x + y * iw)]
                         : 0;
      return ret;
   };

   unsigned tl = M(left - 1, top - 1);
   unsigned tr = M(right, top - 1);
   unsigned bl = M(left - 1, bottom);
   unsigned br = M(right, bottom);

   assert(br + tl >= tr + bl);

   unsigned ret = unsigned(int32_t(br + tl) - int32_t(tr + bl));

   return ret;
}

} // namespace perceive
