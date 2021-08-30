
#pragma once

#include "vector.hpp"

namespace perceive
{
// Vector3d fit_2d_line(const Vector3d * Xs, unsigned N, unsigned stride);
Vector3r fit_2d_line(const Vector3r* Xs, unsigned N, unsigned stride);
Vector3 fit_2d_line(const Vector3* Xs, unsigned N, unsigned stride);

Vector3 fit_line(const vector<Vector2>& Xs, const vector<unsigned>& inds);

// ------------------------------------------------------------- is-collinear-R2
// Test if three points are collinear
template<typename T, typename U, typename V>
bool is_collinear_R2(const T& p, const U& q, const V& r, real epsilon = 1e-9)
{
   return (q(1) - p(1)) * (r(0) - q(0)) - (q(0) - p(0)) * (r(1) - q(1))
          < epsilon;
}

// --------------------------------------------------------------- on-segment-R2
// Returns TRUE if collinear point 'r' is on the line segment {p, q}.
template<typename T, typename U, typename V>
bool on_segment_R2(const T& p, const U& q, const V& r, real epsilon = 1e-9)
{
   auto ind = 0, o_ind = 1;
   auto denom = q(ind) - p(ind);

   auto f_abs = [&](auto v) -> bool {
      if constexpr(std::is_integral_v<decltype(denom)>) {
         return std::abs(v);
      } else {
         return fabs(v);
      }
   };

   if(f_abs(denom) < epsilon) {
      ind   = 1;
      o_ind = 0;
      denom = q(ind) - p(ind);
   }

   auto k              = (r(ind) - p(ind)) / denom;
   auto in_range       = (k >= 0.0 and k <= 1.0);
   auto collinear_dist = f_abs(r(o_ind) - p(o_ind) - k * (q(o_ind) - p(o_ind)));

   return in_range and collinear_dist < epsilon;
}

} // namespace perceive
