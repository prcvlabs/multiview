
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/io/struct-meta.hpp"

namespace perceive::disparity
{
class TriclopsDisparity
{
 public:
   // -- Parameters --
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      bool rectify{false};
      bool lowpass{true};
      bool subpixel{true};
      bool subpixel_validation{true};
      bool edge_correlation{true};
      int min_disp{1};
      int max_disp{128};
      int edge_mask{11};
      int stereo_mask{17};
      bool texture_validation{true};
      bool surface_validation{true};
      int surface_validation_size{400};
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
