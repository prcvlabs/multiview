
#pragma once

#include "skeleton-2d-info.hpp"

namespace perceive
{
struct Skeleton2DInterpolation
{
   struct Params
   {
      int patch_w                         = 0;
      int patch_h                         = 0;
      float projective_distance_threshold = 0.0; // In projective space =(
      unsigned frame_t_window             = 0; // max 'dt' for an interpolation
   };

   float lab_score      = fNAN;
   float geometry_score = fNAN;

   vector<Skeleton2DInfo> p2d_infos;

   // vector<shared_ptr<const Skeleton2D>> p2ds; // interpolated p2ds
   // vector<vector<LABImage>> patches;          // patches for interpolations

   int n_interpolated_frames() const noexcept { return int(p2d_infos.size()); }
};

bool is_projective_interpolation_candidate(
    const Skeleton2DInterpolation::Params& params,
    const Skeleton2D& p2d0, // pd20.sensor_no() == p2d1.sensor_no()
    const Skeleton2D& p2d1) noexcept;

Skeleton2DInterpolation projective_interpolation(
    const Skeleton2DInterpolation::Params& params,
    const Skeleton2D::Params& p2d_params,
    const Skeleton2D& p2d0, // pd20.sensor_no() == p2d1.sensor_no()
    const Skeleton2D& p2dn,
    const vector<LABImage>& patches0,
    const vector<LABImage>& patchesn,
    const DistortedCamera* dcam_ptr,
    std::function<const LABImage*(int frameno, int sensor_no)> get_lab);

void create_interp_movie(
    const string_view filename,
    const Skeleton2DInfo& info0,
    const Skeleton2DInfo& infoN,
    const Skeleton2DInterpolation& interp,
    std::function<const LABImage*(int frameno, int sensor_no)> get_lab);

} // namespace perceive
