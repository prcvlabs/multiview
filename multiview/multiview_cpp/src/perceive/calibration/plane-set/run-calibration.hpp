
#pragma once

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/euclidean-transform.hpp"

namespace perceive::calibration
{
struct PlaneSetCalibParams
{
   string scene_fname;           // for loading bcams
   string params_fname;          // params
   int bcam_index = 0;           // for bcam in scene
   string image_fname;           // left-right image pair
   string calib_plane_set_fname; // for loading a 'CalibPlaneSet'
   string complete_cps_fname;    // for creating disparity map
   string outdir;                // to save outputs
   EuclideanTransform et0;       // initial extrinsic position of CAM0
   ImageFeatures2d::Params f2dp; // for running SLIC
};

bool run_plane_set_calibration(const PlaneSetCalibParams& params);
} // namespace perceive::calibration
