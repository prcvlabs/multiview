
#pragma once

#include "line-2d.hpp"
#include "perceive/foundation.hpp"
#include "vector.hpp"

namespace perceive
{
// ------------------------------------------------------------------- Angle ABC
// Theta in triangle A->B->C
template<typename T>
inline T
angle_ABC(const Vector3T<T>& A, const Vector3T<T>& B, const Vector3T<T>& C)
{
   const auto v1        = (A - B).normalised();
   const auto v2        = (B - C).normalised();
   const auto cos_theta = std::clamp(dot(v1, v2), T(0.0), T(1.0));
   return acos(cos_theta);
}

// Theta in triangle A->B->C, such that theta < 90 degrees
template<typename T>
inline T
angle_ABC2(const Vector3T<T>& A, const Vector3T<T>& B, const Vector3T<T>& C)
{
   const auto v1        = (A - B).normalised();
   const auto v2        = (B - C).normalised();
   const auto cos_theta = std::clamp(std::fabs(dot(v1, v2)), T(0.0), T(1.0));
   return acos(cos_theta);
}

// ------------------------------------------------------------------- Angle ABC
// Theta in triangle A->B->C
template<typename Itr, typename T>
inline Vector2T<T> line_segment_polygon_isect(const Vector2T<T>& A,
                                              const Vector2T<T>& B,
                                              Itr begin,
                                              Itr end) noexcept
{
   if(begin == end) return Vector2T<T>::nan();

   auto ii = begin;
   auto jj = std::next(ii);
   do {
      const auto C
          = homgen_P2_to_R2(segment_segment_intersection(A, B, *ii++, *jj++));
      if(C.is_finite()) return C;
   } while(jj != end);

   return Vector2T<T>::nan();
}

// ---------------------------------------------------- circle-intersection-area
// For two circles of the same radius
real circles_intersection_area(const Vector2& u,
                               const Vector2& v,
                               const real r) noexcept;

real circles_union_area(const Vector2& u,
                        const Vector2& v,
                        const real r) noexcept;

real circles_intersection_on_union(const Vector2& u,
                                   const Vector2& v,
                                   const real r) noexcept;

real circles_intersection_area(const real d, // distance between centres
                               const real r) noexcept; // radius of circles

real circles_union_area(const real d, const real r) noexcept;

real circles_intersection_on_union(const real d, const real r) noexcept;

// --------------------------------------------------------------- Triangle Area
// Signed area of a triangle
template<typename T>
T signed_triangle_area(const Vector2T<T>& A,
                       const Vector2T<T>& B,
                       const Vector2T<T>& C)
{
   return 0.5 * ((B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y));
}

template<typename T>
T triangle_area(const Vector2T<T>& A,
                const Vector2T<T>& B,
                const Vector2T<T>& C)
{
   return fabs(triangle_area(A, B, C));
}

// ----------------------------------------------------------- Point in Triangle

inline bool point_in_triangle(const Vector2& s,
                              const Vector2& a,
                              const Vector2& b,
                              const Vector2& c);

template<typename T>
inline bool point_in_triangle(const Vector3T<T>& X,
                              const Vector3T<T>& A, // 3D triangle
                              const Vector3T<T>& B,
                              const Vector3T<T>& C);

inline bool triangle_ray_intersection(const Vector3& A,
                                      const Vector3& B,
                                      const Vector3& C,
                                      const Vector3& u, // ray point
                                      const Vector3& v) // ray point
{
   auto plane = Vector4(A, B, C);
   return point_in_triangle(plane_ray_intersection(plane, u, v), A, B, C);
}

template<typename T>
inline bool line_segment_intersects_triangle(const Vector3T<T>& A,
                                             const Vector3T<T>& B,
                                             const Vector3T<T>& C,
                                             const Vector3T<T>& u,
                                             const Vector3T<T>& v) noexcept
{
   static_assert(std::is_floating_point<T>::value);

   const auto p3 = Vector4T<T>(A, B, C);
   const auto t  = plane_ray_intersection_t(p3, u, v);
   if(t < T(0.0) || t > T(1.0)) return false;
   const auto X = u + t * (v - u);
   return point_in_triangle(X, A, B, C);
}

template<typename T>
inline bool line_segment_intersects_quad3d(const Vector3T<T>& A,
                                           const Vector3T<T>& B,
                                           const Vector3T<T>& C,
                                           const Vector3T<T>& D,
                                           const Vector3T<T>& u,
                                           const Vector3T<T>& v) noexcept
{
   static_assert(std::is_floating_point<T>::value);

   const auto p3 = Vector4T<T>(A, B, C);
   const auto t  = plane_ray_intersection_t(p3, u, v);
   if(t < T(0.0) || t > T(1.0)) return false;
   const auto X = u + t * (v - u);
   return point_in_triangle(X, A, B, C) or point_in_triangle(X, C, D, A);
}

// B -- C
// |    |
// A -- D
inline bool quad_ray_intersection(const Vector3& A,
                                  const Vector3& B,
                                  const Vector3& C,
                                  const Vector3& D,
                                  const Vector3& u, // ray point
                                  const Vector3& v) // ray point
{
   return triangle_ray_intersection(A, B, C, u, v)
          or triangle_ray_intersection(A, C, D, u, v);
}

// 1 -- 2
// |    |
// 0 -- 3
inline bool quad_ray_intersection(const array<Vector3, 4> X,
                                  const Vector3& u, // ray point
                                  const Vector3& v) // ray point
{
   return triangle_ray_intersection(X[0], X[1], X[2], u, v)
          or triangle_ray_intersection(X[0], X[2], X[3], u, v);
}

// ------------------------------------------------------------ Point in Polygon

template<typename T, typename Itr>
inline bool point_in_polygon(const T& p, Itr begin, Itr end);

template<typename Itr>
inline void points_in_polygon(Itr begin,
                              Itr end,
                              std::function<void(typename Itr::value_type)> f);

// ---------------------------------------------------------------- Polygon Area

template<typename Itr> inline real polygon_area(Itr begin, Itr end);

template<typename Itr, typename F>
inline real polygon_area(Itr begin, Itr end, F get_point_func);

// ----------------------------------------------------------------- Convex Hull

template<typename InputIt>
inline void
andrews_convex_hull(InputIt begin, InputIt end, std::vector<Vector2>& out);

template<typename InputIt>
inline void
andrews_convex_hull(InputIt begin, InputIt end, std::vector<unsigned>& out);

// ------------------------------------------------------ Hull-line Intersection

template<typename InputItr>
inline void hull_line_intersections(InputItr begin,
                                    InputItr end,
                                    const Vector3& l,
                                    vector<Vector2>& isects);

// ----------------------------------------- Convex Hull union/intersection area

template<typename Itr>
inline real convex_hull_union_area(Itr begin0, Itr end0, Itr begin1, Itr end1);

template<typename Itr>
inline vector<Vector2>
convex_hull_intersection(Itr begin0, Itr end0, Itr begin1, Itr end1);

template<typename Itr>
inline real
convex_hull_intersection_over_union(Itr begin0, Itr end0, Itr begin1, Itr end1);

// ------------------------------------------------------------- Implementations

inline bool point_in_triangle(const Vector2& s,
                              const Vector2& a,
                              const Vector2& b,
                              const Vector2& c)
{
   double as_x = s.x - a.x;
   double as_y = s.y - a.y;
   bool s_ab   = (b.x - a.x) * as_y - (b.y - a.y) * as_x > 0;
   if(((c.x - a.x) * as_y - (c.y - a.y) * as_x > 0) == s_ab) return false;
   if(((c.x - b.x) * (s.y - b.y) - (c.y - b.y) * (s.x - b.x) > 0) != s_ab)
      return false;
   return true;
}

template<typename T>
inline bool point_in_triangle(const Vector3T<T>& X,
                              const Vector3T<T>& A,
                              const Vector3T<T>& B,
                              const Vector3T<T>& C)
{
   // We want to find 'x', 'y', that make:
   //
   //     x(B - A) + y(C - A) = (X - A)
   //
   // If x >= 0 && y >= 0, and x + y <= 1, then 'X' is inside 'ABC'
   // We have two unknowns and three equations, so use Moore-Penrose

   static_assert(std::is_floating_point<T>::value);

   Vector3T<T> b = (B - A);
   Vector3T<T> c = (C - A);
   Vector3T<T> p = (X - A);

   T dot_bc  = dot(b, c);
   T quad_b  = quadrance(b);
   T quad_c  = quadrance(c);
   T det_inv = 1.0 / (quad_b * quad_c - dot_bc * dot_bc);

   // Values of the Moore-Penrose pseduo-inverse of matrix [b|c]
   T Mp00 = det_inv * (quad_c * b.x - dot_bc * c.x);
   T Mp01 = det_inv * (quad_c * b.y - dot_bc * c.y);
   T Mp02 = det_inv * (quad_c * b.z - dot_bc * c.z);
   T Mp10 = det_inv * (quad_b * c.x - dot_bc * b.x);
   T Mp11 = det_inv * (quad_b * c.y - dot_bc * b.y);
   T Mp12 = det_inv * (quad_b * c.z - dot_bc * b.z);

   T x = (Mp00 * p.x) + (Mp01 * p.y) + (Mp02 * p.z);
   T y = (Mp10 * p.x) + (Mp11 * p.y) + (Mp12 * p.z);

   if(x >= 0 && y >= 0 && (x + y) <= 1.0) return true;
   return false;

   // if constexpr(false) {
   //    bool sanity_checks = true;
   //    if(sanity_checks) { // Check the pseudo-inverse
   //       Eigen::Matrix<T, 3, 2> M;
   //       M << b.x, c.x, b.y, c.y, b.z, c.z;

   //       Eigen::Matrix<T, 2, 3> Mp;
   //       Mp(0, 0) = Mp00;
   //       Mp(0, 1) = Mp01;
   //       Mp(0, 2) = Mp02;
   //       Mp(1, 0) = Mp10;
   //       Mp(1, 1) = Mp11;
   //       Mp(1, 2) = Mp12;

   //       Eigen::Matrix<T, 2, 2> MM = Mp * M;
   //       bool problem              = false;
   //       for(uint i = 0; i < 2; ++i)
   //          for(uint j = 0; j < 2; ++j)
   //             if(!(fabs(MM(i, j) - (i == j ? 1.0 : 0.0)) < 1e-9))
   //                problem = true;
   //       if(problem) {
   //          cout << "Moore-Penrose pseudo inverse was no good" << endl <<
   //          endl; b.print("b = "); c.print("c = "); cout << format("det_inv =
   //          {}", det_inv) << endl; cout << format("quad_b = {}", quad_b) <<
   //          endl; cout << format("quad_c = {}", quad_c) << endl; cout <<
   //          format("dot_bc = {}", dot_bc) << endl; cout << "M = " << endl <<
   //          M << endl << endl; cout << "Mp = " << endl << Mp << endl << endl;
   //          cout << "MM = " << endl << MM << endl << endl;
   //          // FATAL("kBAM!");
   //       }
   //    }
   // }
}

template<typename T, typename Itr>
inline bool point_in_polygon(const T& p, Itr begin, Itr end)
{
   bool ret = false;

   auto jj0 = begin;
   std::advance(jj0, std::distance(begin, end) - 1);

   // Test vertices and line segments
   for(auto jj = jj0, ii = begin; ii != end; jj = ii++) {
      if(p(0) == (*ii)(0) and p(1) == (*ii)(1)) return true;
      if(on_segment_R2(*ii, *jj, p)) return true;
   }

   // Test the volume of the polygon
   for(auto jj = jj0, ii = begin; ii != end; jj = ii++)
      if((((*ii)(1) > p(1)) != ((*jj)(1) > p(1)))
         and (p(0) < (((*jj)(0) - (*ii)(0)) * (p(1) - (*ii)(1))
                          / ((*jj)(1) - (*ii)(1))
                      + (*ii)(0))))
         ret = !ret;

   return ret;
}

template<typename Itr>
inline void points_in_polygon(Itr begin,
                              Itr end,
                              std::function<void(typename Itr::value_type)> f)
{
   using VectorT = typename Itr::value_type;
   using T       = typename VectorT::value_type;

   AABBT<T> aabb = AABBT<T>::minmax();
   for(auto ii = begin; ii != end; ++ii) aabb.union_point(*ii);

   const int l = int(std::floor(aabb.left));
   const int t = int(std::floor(aabb.top));
   const int r = int(std::ceil(aabb.right));
   const int b = int(std::ceil(aabb.bottom));

   for(auto y = t; y <= b; ++y)
      for(auto x = l; x <= r; ++x)
         if(point_in_polygon(VectorT(T(x), T(y)), begin, end))
            f(VectorT(T(x), T(y)));
}

template<typename Itr> inline real polygon_area(Itr begin, Itr end)
{
   real area{0.0};

   auto jj = begin;
   for(auto ii = begin + 1; ii != end; ++ii, ++jj)
      area += perp_dot((*jj)(0), (*jj)(1), (*ii)(0), (*ii)(1));
   area += perp_dot((*jj)(0), (*jj)(1), (*begin)(0), (*begin)(1));

   return fabs(area) * 0.5;
}

template<typename Itr, typename F>
inline real polygon_area(Itr begin, Itr end, F point)
{
   real area{0.0};

   auto jj = begin;
   for(auto ii = begin + 1; ii != end; ++ii, ++jj) {
      const auto& a = point(*jj);
      const auto& b = point(*ii);
      area += perp_dot(a(0), a(1), b(0), b(1));
   }

   const auto& a = point(*jj);
   const auto& b = point(*begin);
   area += perp_dot(a(0), a(1), b(0), b(1));

   return fabs(area) * 0.5;
}

template<typename InputIt>
inline void
andrews_convex_hull(InputIt begin,
                    InputIt end,
                    // this just means: vector<whatever type *begin is>
                    std::vector<Vector2>& out)
{
   std::vector<unsigned> out_ind;
   andrews_convex_hull(begin, end, out_ind);
   out.resize(out_ind.size());
   for(unsigned i = 0; i < out_ind.size(); ++i) out[i] = *(begin + out_ind[i]);
}

template<typename InputIt>
inline void
andrews_convex_hull(InputIt begin, InputIt end, std::vector<unsigned>& out)
{
   const unsigned n = unsigned(std::distance(begin, end));

   auto point = [&](unsigned i) { return *(begin + i); };

   std::vector<unsigned> bufidx(n);
   std::iota(bufidx.begin(), bufidx.end(), 0);
   sort(bufidx.begin(), bufidx.end(), [&](unsigned i, unsigned j) -> bool {
      const auto& a = point(i);
      const auto& b = point(j);
      return a(0) < b(0) || (a(0) == b(0) && a(1) < b(1));
   });

   if(out.size() < n + 1) out.resize(n + 1);

   unsigned* H = &out[0];
   unsigned k  = 0; // number of hull points used

   // Degenerate case
   if(n <= 3) {
      out.resize(n);
      std::iota(out.begin(), out.end(), 0);
      return;
   }

   // Are we "turning" clockwise or anti-clockwise?
   auto turn = [&](unsigned i) -> bool {
      const auto& a = point(H[k - 1]);
      const auto& b = point(H[k - 2]);
      const auto& c = point(bufidx[i]);
      return perp_dot(a(0) - b(0), a(1) - b(1), c(0) - b(0), c(1) - b(1))
             <= real(0.0);
   };

   // Find the lower hull
   for(unsigned i = 0; i < n; ++i) {
      while(k >= 2 && turn(i)) --k;
      assert(k >= 0 && k < n);
      H[k++] = bufidx[i];
   }

   // At this stage, we could have k + 1 == n
   if(k >= n + 1) FATAL(format("k+1 == {}, n = {}", k + 1, n));

   // Find the upper hull
   for(int i = int(n) - 2, t = int(k) + 1; i >= 0; --i) {
      while(int(k) >= t && turn(unsigned(i))) --k;
      assert(k >= 0 && k <= n);
      H[k++] = bufidx[size_t(i)];
   }

   // Shrink to fit the output
   out.resize(k);
}

template<typename InputItr>
inline void hull_line_intersections(InputItr begin,
                                    InputItr end,
                                    const Vector3& l,
                                    vector<Vector2>& isects)
{
   auto ii = begin;
   auto jj = ii;
   std::advance(jj, std::distance(begin, end) - 1);

   isects.clear();
   for(; ii != end; jj = ii++) {
      auto s = to_homgen_line(*jj, *ii); // create the line
      auto x = line_line_isect(s, l);    // get intersection
      if(on_segment_R2(*jj, *ii, x)) isects.push_back(x);
   }

   // We will have double-ups if the line intersects a hull vertex
   std::sort(isects.begin(), isects.end());
   auto last = std::unique(
       isects.begin(), isects.end(), [&](const auto& a, const auto& b) {
          return (a - b).quadrance() < 1e-9;
       });
   isects.erase(last, isects.end());
}

// ----------------------------------------- Convex Hull union/intersection area

template<typename Itr>
inline real convex_hull_union_area(Itr begin0, Itr end0, Itr begin1, Itr end1)
{
   auto a0    = polygon_area(cbegin(begin0), cend(end0));
   auto a1    = polygon_area(cbegin(begin1), cend(end1));
   auto isect = convex_hull_intersection(begin0, end0, begin1, end1);
   auto ia    = polygon_area(cbegin(isect), cend(isect));
   return a0 + a1 - ia;
}

template<typename Itr>
inline real
convex_hull_intersection_over_union(Itr begin0, Itr end0, Itr begin1, Itr end1)
{
   auto a0    = polygon_area(cbegin(begin0), cend(end0));
   auto a1    = polygon_area(cbegin(begin1), cend(end1));
   auto isect = convex_hull_intersection(begin0, end0, begin1, end1);
   auto ia = isect.size() < 3 ? 0.0 : polygon_area(cbegin(isect), cend(isect));
   return ia / (a0 + a1 - ia);
}

template<typename Itr>
inline vector<Vector2>
convex_hull_intersection(Itr begin0, Itr end0, Itr begin1, Itr end1)
{
   using T = Vector2;

   // First find the convex hull of the union of both input hulls
   std::vector<T> union_hull;
   std::vector<T> all_points;
   auto n_points
       = size_t(std::distance(begin0, end0) + std::distance(begin1, end1));
   union_hull.reserve(n_points);
   all_points.reserve(n_points);
   all_points.insert(all_points.end(), begin0, end0);
   all_points.insert(all_points.end(), begin1, end1);
   andrews_convex_hull(cbegin(all_points), cend(all_points), union_hull);

   auto n_in_points
       = size_t(std::distance(begin0, end0) + std::distance(begin1, end1));
   std::vector<T> points;
   points.reserve(n_in_points * 2);

   // Add interior vertices from hull0
   for(auto ii = begin0; ii != end0; ++ii)
      if(point_in_polygon(*ii, begin1, end1)) points.push_back(*ii);

   // Add interior vertices from hull1
   for(auto ii = begin1; ii != end1; ++ii)
      if(point_in_polygon(*ii, begin0, end0)) points.push_back(*ii);

   // Add intersection points -- this isn't the fastest method =0
   std::vector<Vector2> isects;
   auto jj = begin0;
   std::advance(jj, std::distance(begin0, end0) - 1);
   for(auto ii = begin0; ii != end0; jj = ii++) {
      auto l = to_homgen_line(*ii, *jj);
      hull_line_intersections(begin1, end1, l, isects);
      for(const auto& X : isects)
         if(point_in_polygon(X, begin0, end0)) points.push_back(X);
   }

   vector<Vector2> out;
   if(points.size() > 2) andrews_convex_hull(cbegin(points), cend(points), out);
   return out;
}

} // namespace perceive
