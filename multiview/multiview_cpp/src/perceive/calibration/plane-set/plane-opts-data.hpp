
#pragma once

#include <opencv2/core/core.hpp>

#include "calib-plane-set.hpp"
#include "run-calibration.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/image-container.hpp"
//#include "perceive/tweaker/tweaker-results.hpp"

namespace perceive::calibration
{
struct PlaneOpsData
{
   string outdir; // to save outputs

   int w = 0;
   int h = 0;

   // TweakerParams params;
   string scene_key;
   BinocularCameraInfo bcam_info;
   ImageFeatures2d::Params f2dp;               // for running SLIC
   array<cv::Mat, 2> cv_im;               // input images (OpenCV)
   array<cv::Mat, 2> cv_grey;             // grey input images (OpenCV)
   array<ImageFeatures2d, 2> f2d;         // image features on 'im'
   array<CachingUndistortInverse, 2> cu;  // distort/undistort
   array<IntImage, 2> cp_lookup;          // lookup cp-index
   array<IntImage, 2> complete_cp_lookup; // lookup cp-index
   EuclideanTransform et0;                     // initial position of CAM0
   CalibPlaneSet cps;                          // the plane sets of interest
   CalibPlaneSet complete_cps;                 // for creating disparity
   ParallelJobSet pjobs;                       // for parallel execution

   bool init(const PlaneSetCalibParams& params);
   // bool init(const TweakerResults& dat,
   //           int cam_ind,
   //           const CalibPlaneSet& cps,
   //           const CalibPlaneSet& complete_cps);

 private:
   void finish_init();
};

ARGBImage make_slic_plane_image(const PlaneOpsData& data, int img_ind);

} // namespace perceive::calibration
