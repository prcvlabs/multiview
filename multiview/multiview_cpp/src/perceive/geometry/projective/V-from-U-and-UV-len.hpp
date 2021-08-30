
#pragma once

namespace perceive
{
// @param `k_u` Distance between U and cam center C, i.e., k_u = (U - C).norm();
// @param `cos_phi` Of angle between U and V. i.e., dot(U, V) / \U\|V\
// @param `len` Length of the vector UV, i.e., len == (U - V).norm()

template<typename T>
inline std::pair<T, T> calc_k_v(const T k_u, const T cos_phi, const T len)
{
   static_assert(std::is_floating_point<T>::value);
   const T b_ac = square(k_u) * (square(cos_phi) - T(1.0)) + square(len);
   if(b_ac >= T(0.0)) {
      const T a         = k_u * cos_phi;
      const T sqrt_b_ac = std::sqrt(b_ac);
      return {a + sqrt_b_ac, a - sqrt_b_ac};
   }
   return {std::numeric_limits<T>::quiet_NaN(),
           std::numeric_limits<T>::quiet_NaN()};
}

inline std::pair<Vector3f, Vector3f>
V_from_U_and_UV_len(const Vector3f& C,
                    const Vector3f& U,
                    const Vector3f& v_ray, // ray
                    const float len) noexcept
{
   const auto k_u          = (U - C).norm();
   const auto u_ray        = (U - C) / k_u;
   const auto cos_phi      = std::clamp<float>(dot(u_ray, v_ray), -1.0f, 1.0f);
   const auto [k_v0, k_v1] = calc_k_v<float>(k_u, cos_phi, len);
   return {C + k_v0 * v_ray, C + k_v1 * v_ray};
}

inline std::pair<Vector3, Vector3> V_from_U_and_UV_len(const Vector3& C,
                                                       const Vector3& u, // ray
                                                       const Vector3& v, // ray
                                                       const real k_u, //  |C-V|
                                                       const real len) noexcept
{
   assert(std::fabs(u.quadrance() - 1.0) < 1e-6);
   assert(std::fabs(v.quadrance() - 1.0) < 1e-6);
   assert(std::isfinite(k_u));
   assert(std::isfinite(len));
   assert(len > 0.0);
   assert(C.is_finite());

   const real cos_phi = std::clamp<real>(dot(u, v), -1.0, 1.0);
   const auto [a, b]  = calc_k_v(k_u, cos_phi, len);
   return {C + a * v, C + b * v};

   const real b_ac = square(k_u * cos_phi) - square(k_u) + square(len);

   if(b_ac >= 0.0) {
      const real a         = k_u * cos_phi;
      const real sqrt_b_ac = std::sqrt(b_ac);
      return {C + (a + sqrt_b_ac) * v, C + (a - sqrt_b_ac) * v};
   }

   return {Vector3::nan(), Vector3::nan()};
}

} // namespace perceive
