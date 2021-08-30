
#pragma once

#include "vector.hpp"

namespace perceive
{
namespace detail
{
   void set_fit_plane_debug_flag(bool value);
}

Vector4 fit_plane(const std::vector<Vector3>& Xs) noexcept;
Vector4 fit_plane(const std::vector<Vector3r>& Xs) noexcept;
Vector4 fit_plane(const Vector3* Xs, const unsigned n) noexcept;
Vector4 fit_plane(const Vector3r* Xs, const unsigned n) noexcept;
Vector4 fit_plane(std::initializer_list<Vector3> Xs) noexcept;

Vector3 intersection_of_3_planes(const Plane& A,
                                 const Plane& B,
                                 const Plane& C) noexcept;

} // namespace perceive
