#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
// x0 and x1 are NORMALIED coordinates
// R and t are for the 2nd camera.
Vector3 triangulate(const Vector2& x0,
                    const Vector2& x1,
                    const Matrix3r& R,
                    const Vector3& t) noexcept;

// Intersecting rays...
Vector3 intersect_rays(const Vector3& u0,
                       const Vector3& u1,
                       const Vector3& v0,
                       const Vector3& v1);

// Intersect rays, where C0 and C1 are _definitely_ on the epipolar plane,
// and so 'u1' and 'v1' are projected onto the plane that includes:
//           {C0, C1, 0.5 * (u1 + v1)}
Vector3 intersect_rays_2(const Vector3& C0,
                         const Vector3& u1,
                         const Vector3& C1,
                         const Vector3& v1);

// How far are we away from the epipolar geometry in 3D?
real intersect_rays_2_plane_err(const Vector3& C0,
                                const Vector3& u1,
                                const Vector3& C1,
                                const Vector3& v1);

} // namespace perceive
