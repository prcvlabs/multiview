
#pragma once

#include "perceive/geometry/skeleton/skeleton-2d.hpp"

namespace perceive
{
void gl_render_cylinder_result(const Skeleton2D::CylinderResult& cy_ret,
                               const int p2d_ind,
                               const uint32_t kolour,
                               const float alpha);

void gl_render_3d_skeleton(const Skeleton3D& p3d,
                           const float height,
                           const real theta,
                           const uint32_t kolour);

void gl_render_3d_cylinder(const Skeleton3D& p3d,
                           const float height,
                           const int p2d_ind,
                           const uint32_t kolour,
                           const float alpha);

} // namespace perceive
