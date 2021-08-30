
#pragma once

#include "perceive/geometry/vector.hpp"

namespace perceive
{
template<typename T>
inline std::pair<T, T>
UV_on_same_p3_with_len(const Vector3T<T>& u, // u ray
                       const Vector3T<T>& v, // v ray
                       const Vector3T<T>& n, // plane normal
                       const T length)       // Length of (U - V)
{
   assert(std::fabs(u.quadrance() - T(1.0)) < T(1e-6));
   assert(std::fabs(v.quadrance() - T(1.0)) < T(1e-6));
   assert(std::fabs(n.quadrance() - T(1.0)) < T(1e-6));

   const T cos_psi   = std::clamp<T>(dot(u, v), -T(1.0), T(1.0));
   const T cos_theta = std::clamp<T>(dot(u, n), -T(1.0), T(1.0));
   const T cos_phi   = std::clamp<T>(dot(v, n), -T(1.0), T(1.0));

   const T ratio = cos_theta / cos_phi;
   T k_u         = std::sqrt(square(length)
                     / (T(1.0) + square(ratio) - T(2.0) * ratio * cos_psi));
   T k_v         = k_u * ratio;

   if(false) {
      if(std::isfinite(k_u) && std::isfinite(k_v))
         assert(is_close((k_u * u - k_v * v).norm(), length));
   }

   return {k_u, k_v};
}

} // namespace perceive
