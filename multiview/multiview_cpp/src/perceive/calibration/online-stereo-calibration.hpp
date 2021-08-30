
#pragma once

#include "perceive/geometry/euclidean-transform.hpp"

namespace perceive::calibration
{
struct OnlineStereoParams
{
   string camera_id;   // Like C0001001_v8
   string image_fname; // left-right image pair
   string outdir;      // to save outputs

   string cam0_fname;
   string cam1_fname;

   int match_aperture         = 11;
   real match_thres           = 0.30;
   real distorted_y_threshold = 50.0;
   real distorted_x_threshold = 100.0;
   real ransac_threshold      = 2.00; // pixels at f=400 pixels

   bool use_harris    = true;
   bool use_min_eigen = false;

   bool optimize_scharr_match = true;

   EuclideanTransform extrinsic{Vector3{4.174202674378069133354e+00,
                                        2.440705667719924143455e+00,
                                        3.207383151216594008304e+00},
                                Quaternion{-4.405845330199578691044e-01,
                                           -7.540191723100938547830e-01,
                                           4.278185492494328556745e-01,
                                           2.330485914364152644218e-01},
                                1.25};

   vector<std::pair<Vector2, Vector2>> in_corners;
};

bool run_online_stereo_calibration(const OnlineStereoParams& params);
} // namespace perceive::calibration
