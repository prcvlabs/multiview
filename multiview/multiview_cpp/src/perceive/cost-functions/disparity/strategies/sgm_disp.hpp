
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/io/struct-meta.hpp"

namespace perceive::disparity
{
class SGMDisparity
{
 public:
   // -- Parameters --
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      int disparity_size       = 256; // must be: 64, 128, or 256
      bool use_16bit_greyscale = true;
   };

   // -- Calculate --
   // grey smoothed images
   static void calculate(const Params& p,
                         const cv::Mat& grey0,
                         const cv::Mat& grey1,
                         cv::Mat& out) noexcept; // A float matrix
};

} // namespace perceive::disparity
