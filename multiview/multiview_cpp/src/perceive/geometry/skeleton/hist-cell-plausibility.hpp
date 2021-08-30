
#pragma once

#include "skeleton-2d.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
struct DistortedCamera;

// Calculate the plausibility that pose 'p2d' can be in histogram cell 'pos'
struct HistCellPlausibility
{
   // Estimated height at this position
   float height = fNAN;

   // Y0 = project-skeleton onto the floor
   // Yr = vector from (Y0 - cam-centre)
   // Y  = Yr * (X - cam-centre).norm()
   // Then `proj_floor_dist` = (X - Y).norm()
   // And note that Z = 0.5 * (X + Y) bisects line (X-Y) at right-angle
   float proj_floor_dist = fNAN;

   // Distance between hist-cell and camera (for camera that saw skeleton)
   // in `LocalizationData::Params::extrinsic_calibration_error_factor`
   float n_cam_dist = fNAN;

   float normalized_height_prob() const noexcept;
   float normalized_pfd_prob() const noexcept; // proj-floor-dist probability
   float probability() const noexcept;
};
HistCellPlausibility
hist_cell_plausibility(const LocalizationData::Params& p,
                       const Skeleton2D& p2d,
                       const DistortedCamera& dcam,
                       const Vector3& X, // Position, assumes X.z=0.0
                       const bool feedback = false) noexcept;

} // namespace perceive
