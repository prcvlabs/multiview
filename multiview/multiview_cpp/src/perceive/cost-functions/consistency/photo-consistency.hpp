
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

#include "camera-pos-opt-context.hpp"

namespace perceive
{
real colour_consistency(const Vector2& D, // Distorted location we're checking
                        const unsigned ref_camera,
                        const vector<unsigned>& cam_inds, // cams of interest
                        const vector<CameraPosOptContext>& opt_ctxs,
                        const unsigned debug_id = 0,
                        const bool feedback     = false);

// Pass a valid point to sample-stats if you want to calculate such
real colour_consistency(const vector<unsigned>& cam_inds, // cams of interest
                        const vector<CameraPosOptContext>& opt_ctxs,
                        const real threshold,
                        const bool spixel_centers_only = false,
                        SampleStatistics* sample_stats = nullptr);

// Find the best camera positions
real run_camera_optimization(const vector<unsigned>& cam_inds,
                             const BinocularCameraInfo& bcam_info,
                             vector<CameraPosOptContext>& opt_ctxs,
                             const bool use_nelder_mead,
                             const bool feedback = false);

} // namespace perceive
