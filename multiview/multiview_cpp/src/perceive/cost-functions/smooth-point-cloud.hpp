
#pragma once

#include "perceive/geometry/point-cloud.hpp"

namespace perceive
{
struct SmoothPointCloudParams
{
   int polynomial_order = 2;
   real search_radius   = 0.03;
};

PointCloud smooth_pointcloud(const PointCloud& cloud,
                             const SmoothPointCloudParams& params) noexcept;
} // namespace perceive
