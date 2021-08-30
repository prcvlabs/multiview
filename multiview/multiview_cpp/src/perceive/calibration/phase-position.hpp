
#pragma once

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
struct PhasePositionParams
{
   string outdir{"/tmp"s};
   ArucoCube ac;
   bool render_cube_only{false};
   bool animate_cube{false};
   string camera_id{""s};
   vector<string> input_image_fnames{""s};
   vector<ARGBImage> raw_input_images;
   vector<array<ARGBImage, 2>> images;
   BinocularCameraInfo bcam_info;
   real baseline_estimate{0.14854};
   bool estimate_bcam_qt{false};
   bool K_opt{false};
   bool dense_et0{false};
   bool use_pointwise_result{false};
   bool use_fast_distort{false};
   bool quite_mode{false};

   string to_string() const noexcept;
   // friend string str(const PhasePositionParams&) noexcept;
};

struct PhasePositionOptOneImage
{
   string image_fname = ""s;
   array<cv::Mat, 2> cv_im;
   array<cv::Mat, 2> undistorted;
   array<vector<ArucoCube::FaceDetection>, 2> detects;
   array<EuclideanTransform, 2> et0s;
   array<EuclideanTransform, 2> ets;

   bool detection_is_good() const noexcept
   {
      return detects[0].size() >= 2 and detects[1].size() >= 2;
   }
};

struct PhasePositionOptDetails
{
   PhasePositionParams p;
   ArucoCube ac;
   Matrix3r K;
   array<CachingUndistortInverse, 2> cus;
   array<cv::Mat, 6> face_ims;
   vector<PhasePositionOptOneImage> imgs;
};

bool run_phase_position_calibration(PhasePositionParams&) noexcept;

} // namespace perceive::calibration
