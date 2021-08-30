
#pragma once

#include "perceive/geometry/vector.hpp"

namespace perceive
{
// Let [n | d] be a symmetry plane, where 'n' is a normalized unit vector.
// Let 'u' and 'v' be normalized vectors that run from the camera center
// through the points 'U' and 'V' in 3D, where 'U' and 'V' are mirror symmetric
// about the symmetry plane.
// Let cos(theta) = dot(u, n)
// Let cos(phi)   = dot(v, n)
// Then
// ||V|| = -2d sin(theta) / sin(theta + phi)
// ||U|| = -2d sin(phi)   / sin(theta + phi)
// ALSO
// ||V||/||U|| = sin(theta) / sin(phi)
//
// NOTE: the degenerate case is when `C` is in the symmetry plane `p3`
inline std::pair<Vector3, Vector3>
recover_mirror_symmetric_pair(const Vector3& C, // camera centre
                              const Plane& p3,  // symmetry plane
                              const Vector3& u,
                              const Vector3& v) noexcept
{
   assert(std::fabs(p3.xyz().quadrance() - 1.0) < 1e-6);
   assert(std::fabs(u.quadrance() - 1.0) < 1e-6);
   assert(std::fabs(v.quadrance() - 1.0) < 1e-6);

   const auto cos_theta = std::clamp<real>(dot(u, p3.xyz()), -1.0, 1.0);
   const auto cos_phi   = std::clamp<real>(dot(v, p3.xyz()), -1.0, 1.0);
   const auto theta     = acos(cos_theta);
   const auto phi       = acos(cos_phi);
   const auto sin_theta_plus_phi_inv = 1.0 / sin(theta + phi);
   const auto dt                     = p3.d() + dot(C, p3.xyz());

   const auto k_u = -2.0 * dt * sin(phi) * sin_theta_plus_phi_inv;
   const auto k_v = -2.0 * dt * sin(theta) * sin_theta_plus_phi_inv;

   return {C + k_u * u, C + k_v * v};
}

// template<typename T>
// inline std::pair<Vector3T<T>, Vector3T<T>>
// recover_mirror_symmetric_pair(const Vector3T<T>& C, // camera centre
//                               const Vector3T<T>& n, // symmetry plane normal
//                               const T len,          // len == (U - V).norm()
//                               const Vector3T<T>& u,
//                               const Vector3T<T>& v) noexcept
// {
//    assert(std::fabs(n.quadrance() - T(1.0)) < T(1e-6));
//    assert(std::fabs(u.quadrance() - T(1.0)) < T(1e-6));
//    assert(std::fabs(v.quadrance() - T(1.0)) < T(1e-6));

//    const auto cos_theta = std::clamp<T>(dot(u, n), -T(1.0), T(1.0));
//    const auto cos_phi   = std::clamp<T>(dot(v, n), -T(1.0), T(1.0));
//    const auto cos_psi   = std::clamp<T>(dot(v, n), -T(1.0), T(1.0));

//    const auto theta = std::acos(cos_theta);
//    const auto phi   = std::acos(cos_phi);

//    const auto sin_theta = std::sin(theta);
//    const auto sin_phi   = std::sin(phi);

//    const auto val = T(1.0) + square(sin_theta / sin_phi)
//                     - T(2.0) * sin_theta * cos_psi / sin_phi;
//    const auto k_u = len / std::sqrt(val);
//    const auto k_v = k_u * sin_theta / sin_phi;

//    return {C + k_u * u, C + k_v * v};
// }

} // namespace perceive
