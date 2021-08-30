
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/io/struct-meta.hpp"

namespace perceive::disparity
{
class StereoSGBMDisparity
{
 public:
   // -- Parameters --
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      // OpenCV Stereo Algorithm
      // @see
      // http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

      int sg_SAD_window_size{13};
      int sg_12_max_diff{596};
      int sg_min_disparity{1};
      int sg_num_disparities{96};
      int sg_speckle_window_size{600};
      int sg_speckle_range{250};
      int sg_prefilter_cap{54};
      int sg_uniqueness_ratio{3};
      int sg_mode{3};
      int sg_p1{22};
      int sg_p2{1000};
   };

   // -- Calculate --
   // grey smoothed images
   static void calculate(const Params& p,
                         const bool l_is_ref_image,
                         const cv::Mat& grey0,
                         const cv::Mat& grey1,
                         cv::Mat& out) noexcept; // A float matrix
};

} // namespace perceive::disparity
