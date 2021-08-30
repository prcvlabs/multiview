
#pragma once

namespace perceive
{
template<typename T>
inline std::pair<T, T>
ray_sphere_intersection(const Vector3T<T>& C, // camera center
                        const Vector3T<T>& ray,
                        const Vector3T<T>& S, // sphere centere
                        const T radius) noexcept
{
   static_assert(std::is_floating_point<T>::value);

   // A quadratic equation
   const auto CS = (C - S);

   const T a = dot(ray, ray); // Should be `1`,
   const T b = T(2.0) * dot(CS, ray);
   const T c = dot(CS, CS) - square(radius);

   return quadratic_formula(a, b, c);
}

} // namespace perceive
