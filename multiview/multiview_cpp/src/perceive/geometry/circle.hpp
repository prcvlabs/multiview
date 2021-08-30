
#pragma once

namespace perceive
{
template<typename T>
inline T intersection_area(const Vector2T<T>& C0,
                           const T r0,
                           const Vector2T<T>& C1,
                           const T r1) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   Expects(r0 >= T(0.0));
   Expects(r1 >= T(0.0));

   const auto d = (C1 - C0).norm();
   if(d < r0 + r1) {
      const auto r02 = square(r0);

      const auto a = square(r0);
      const auto b = square(r1);
      const auto x = (a - b + square(d)) / (T(2.0) * d);
      const auto z = square(x);
      const auto y = std::sqrt(a - z);

      if(d <= std::abs(r1 - r0)) return T(M_PI) * std::min(a, b);

      return a * std::asin(y / r0) + b * std::asin(y / r1)
             - y * (x + std::sqrt(z + b - a));
   }

   return T(0.0);
}

template<typename T>
inline T union_area(const Vector2T<T>& C0,
                    const T r0,
                    const Vector2T<T>& C1,
                    const T r1) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return T(M_PI) * square(r0) + T(M_PI) * square(r1)
          - intersection_area(C0, r0, C1, r1);
}

template<typename T>
inline T intersection_over_union(const Vector2T<T>& C0,
                                 const T r0,
                                 const Vector2T<T>& C1,
                                 const T r1) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   const auto isect = intersection_area(C0, r0, C1, r1);
   return isect / (T(M_PI) * square(r0) + T(M_PI) * square(r1) - isect);
}

} // namespace perceive
