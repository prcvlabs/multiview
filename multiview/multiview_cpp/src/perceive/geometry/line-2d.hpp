
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
   using value_type = decltype(p(0));
   if constexpr(std::is_integral<value_type>::value) {
      auto ind = 0, o_ind = 1;
      auto denom = q(ind) - p(ind);
      if(abs(denom) < epsilon) {
         ind   = 1;
         o_ind = 0;
         denom = q(ind) - p(ind);
      }

      auto k        = (r(ind) - p(ind)) / denom;
      auto in_range = (k >= 0.0 and k <= 1.0);
      auto collinear_dist
          = abs(r(o_ind) - p(o_ind) - k * (q(o_ind) - p(o_ind)));

      return in_range and collinear_dist < epsilon;
   } else {
      auto ind = 0, o_ind = 1;
      auto denom = q(ind) - p(ind);
      if(fabs(denom) < epsilon) {
         ind   = 1;
         o_ind = 0;
         denom = q(ind) - p(ind);
      }

      const auto numerator = r(ind) - p(ind);
      const auto k = (denom != 0.0) ? numerator / denom : value_type(NAN);
      const auto in_range = (k >= 0.0 and k <= 1.0);
      const auto collinear_dist
          = fabs(r(o_ind) - p(o_ind) - k * (q(o_ind) - p(o_ind)));

      return in_range and collinear_dist < epsilon;
   }
}

// ---------------------------------------------------------- dist-to-segment_P2
// Returns distance of 'C' to segment 'AB'. If 'C' doesn't project
// onto the segment, then NAN is returned.
template<typename T>
T dist_to_segment_P2(const Vector3T<T>& A,
                     const Vector3T<T>& B,
                     const Vector3T<T>& C) noexcept
{
   static_assert(std::is_floating_point<T>::value);

   const auto ab = cross(A, B).normalised_line();

   // Get the lines through A and B, orthogonal to 'ab'
   const auto aa = Vector3T<T>(-ab.y, ab.x, ab.y * A.x - ab.x * A.y);
   const auto bb = Vector3T<T>(-ab.y, ab.x, ab.y * B.x - ab.x * B.y);

   // LOG_ERR(format("A = {}, B = {}, C = {}",
   //                str(homgen_P2_to_R2(A)),
   //                str(homgen_P2_to_R2(B)),
   //                str(homgen_P2_to_R2(C))));
   // LOG_ERR(format("aa.dot(A) = {}", aa.dot(A)));
   // LOG_ERR(format("bb.dot(B) = {}", bb.dot(B)));
   // LOG_ERR(format("ab.dot(C) = {}", ab.dot(C)));
   // LOG_ERR(format("aa.dot(B) = {}, aa.dot(C) = {}", aa.dot(B), aa.dot(C)));
   // LOG_ERR(format("bb.dot(A) = {}, bb.dot(C) = {}", bb.dot(A), bb.dot(C)));

   // We: A --- C ---- B, which means
   //  * 'A' and 'C' are on the same side of bb
   //  * 'B' and 'C' are on teh same side of aa
   const auto aa_dot_c = aa.dot(C);
   const auto bb_dot_c = bb.dot(C);

   const bool a_and_c = (bb_dot_c <= T(0.0)) == (bb.dot(A) <= T(0.0));
   const bool b_and_c = (aa_dot_c <= T(0.0)) == (aa.dot(B) <= T(0.0));

   const auto ret
       = (aa_dot_c == T(0.0) or bb_dot_c == T(0.0)) or (a_and_c == b_and_c)
             ? ab.dot(C)
             : dNAN;

   // LOG_ERR(format("ac == bc is ({} == {}) = {}, ret = {}",
   //                str(a_and_c),
   //                str(b_and_c),
   //                str(a_and_c == b_and_c),
   //                ret));

   return ret;
}

// ------------------------------------------------ segment-segment intersection
//
template<typename T>
Vector3T<T> segment_segment_intersection(const Vector2T<T>& a,
                                         const Vector2T<T>& b,
                                         const Vector2T<T>& c,
                                         const Vector2T<T>& d) noexcept
{
   const auto A = Vector3T<T>{a(0), a(1), T(1.0)};
   const auto B = Vector3T<T>{b(0), b(1), T(1.0)};
   const auto C = Vector3T<T>{c(0), c(1), T(1.0)};
   const auto D = Vector3T<T>{d(0), d(1), T(1.0)};

   const auto ab = cross(A, B).normalised_line();
   const auto cd = cross(C, D).normalised_line();

   const auto E = cross(ab, cd);

   const auto ab_dist_e = dist_to_segment_P2(A, B, E);
   const auto cd_dist_e = dist_to_segment_P2(C, D, E);

   return std::isfinite(ab_dist_e) and std::isfinite(cd_dist_e)
              ? E
              : Vector3T<T>::nan();
}

} // namespace perceive
