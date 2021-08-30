
#pragma once

#include "perceive/scene/scene-description.hpp"

namespace perceive
{
void draw_floor_grid_in_situ(ARGBImage& distorted,
                             const AABB bounds,
                             const CachingUndistortInverse& cu,
                             const EuclideanTransform& et) noexcept;

ARGBImage draw_floor_grid(const ARGBImage& distorted,
                          const AABB bounds,
                          const CachingUndistortInverse& cu,
                          const EuclideanTransform& et) noexcept(false);

void draw_floor_cell_in_situ(ARGBImage& distorted,
                             const AABB cell, // the cell to draw
                             const CachingUndistortInverse& cu,
                             const EuclideanTransform& et,
                             const uint32_t fg_kolour,
                             const uint32_t bg_kolour) noexcept;

} // namespace perceive
