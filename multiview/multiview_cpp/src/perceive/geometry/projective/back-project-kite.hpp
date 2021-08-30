
#pragma once

#include "stdinc.hpp"

namespace perceive
{
// Produces the 3d planar rectangle that projects to 'quad-2d'.
// The planar rectangle's centre is 'distance-to-centre' from the camera centre
//
// The 'quad-2d' is a sequence of 4 normalized points (rays) in this order:
//
// b -- c
// |    |
// a -- d
//
// The output 3d quad is a sequence of points in R3.
//
// The theory finds the single kite such that all the cross-products are the
// same. Thus, line AB and DC must be parallel, and so must AD and BC.

array<Vector3, 4> back_project_kite(const array<Vector3, 4>& quad_2d,
                                         double distance_to_center = 1.0);

// ---------- These are just overloads

inline auto back_project_kite(const Vector3& a, const Vector3& b,
                              const Vector3& c, const Vector3& d,
                              double distance_to_center = 1.0)
{
   array<Vector3, 4> quad{{a, b, c, d}};
   return back_project_kite(quad, distance_to_center);
}

inline auto back_project_kite(const Vector3r& a, const Vector3r& b,
                              const Vector3r& c, const Vector3r& d,
                              double distance_to_center = 1.0)
{
   array<Vector3, 4> quad{
       {to_vec3(a), to_vec3(b), to_vec3(c), to_vec3(d)}};
   return back_project_kite(quad, distance_to_center);
}

} // namespace perceive
