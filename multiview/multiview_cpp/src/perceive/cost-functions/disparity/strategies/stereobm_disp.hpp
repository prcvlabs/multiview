
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/io/struct-meta.hpp"

namespace perceive::disparity
{
class StereoBMDisparity
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

      // Stereo-BM _only_
      int bm_SAD_window_size{25};
      int bm_12_max_diff{126};
      int bm_min_disparity{1};
      int bm_num_disparities{96};
      int bm_speckle_window_size{185};
      int bm_speckle_range{111};
      int bm_prefilter_cap{63};
      int bm_prefilter_size{7};
      int bm_prefilter_type{1};
      int bm_block_size{15};
      int bm_texture_threshold{259};
      int bm_uniqueness_ratio{9};
   };

   // -- Calculate --
   // grey smoothed images
   static void calculate(const Params& p,
                         const bool l_is_ref_image,
                         const bool use_cuda_if_available,
                         const cv::Mat& grey0,
                         const cv::Mat& grey1,
                         cv::Mat& out) noexcept; // A float matrix
};

} // namespace perceive::disparity
