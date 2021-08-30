
#pragma once

#include "bone.hpp"
#include "hist-cell-plausibility.hpp"
#include "skeleton-2d.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/geometry/sparse-hist-cell.hpp"

namespace perceive
{
// ---------------------------------------------------------------- calculations
//
vector<Vector3> pose_to_billboard(const Plane& floor,
                                  const CachingUndistortInverse& cu,
                                  const EuclideanTransform& et_inv,
                                  const Skeleton2D& pose,
                                  const real height) noexcept;

Plane calculate_p2d_plane(const Skeleton2D& p2d, const Vector3& X) noexcept;

// ----------------------------------------------------------------- estimations
//
real estimate_pose_direction(const DistortedCamera& dcam,
                             const Skeleton2D& pose) noexcept;

Vector3 estimate_pose_floor_pos(const DistortedCamera& dcam,
                                const Skeleton2D& pose,
                                const real height) noexcept;

// ------------------------------------------------------------ estimate-floor-X

Vector3f estimate_floor_X(const LocalizationData::Params& loc_data_params,
                          const int n_skeletons,
                          const DistortedCamera** dcam_array,
                          const Skeleton2D** s2d_array,
                          const bool feedback = false) noexcept;

// ------------------------------------------------------------------- rendering
//
void trace_skeleton(const Skeleton2D& pose,
                    std::function<void(const Point2&)> f) noexcept;
void render_pose(ARGBImage& im, const Skeleton2D& pose) noexcept;
void render_pose(cv::Mat& im, const Skeleton2D& pose) noexcept;

void draw_billboard(ARGBImage& im,
                    const Skeleton2D& pose,
                    const CachingUndistortInverse& cu,
                    const EuclideanTransform& et_inv,
                    const uint32_t kolour) noexcept(false);
void draw_billboard(cv::Mat& im,
                    const Skeleton2D& pose,
                    const CachingUndistortInverse& cu,
                    const EuclideanTransform& et_inv,
                    const uint32_t kolour) noexcept(false);

} // namespace perceive
