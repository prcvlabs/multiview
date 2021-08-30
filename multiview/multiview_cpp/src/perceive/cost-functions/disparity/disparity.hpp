
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/io/struct-meta.hpp"

#include "strategies/sgm_disp.hpp"
#include "strategies/stereobm_disp.hpp"
#include "strategies/stereosgbm_disp.hpp"
#include "strategies/triclops_disp.hpp"

namespace perceive
{
class Disparity
{
 public:
   enum DisparityMethod : unsigned {
      STEREO_BM = 0, // OpenCV
      STEREO_SGBM,   // OpenCV
      TRICLOPS,
      CUDA_SGM
   };

   // -- Parameters --
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      DisparityMethod disparity_method{STEREO_BM};

      // Prefilter options
      bool use_cuda_if_available{false};
      real presmooth_sigma{2.15};

      disparity::StereoBMDisparity::Params stereobm_params;
      disparity::StereoSGBMDisparity::Params stereosgbm_params;
      disparity::TriclopsDisparity::Params triclops_params;
      disparity::SGMDisparity::Params sgm_params;
   };

   // -- Calculate --
   static void calculate(const Params& p,
                         const bool l_is_ref_image,
                         const cv::Mat& grey0,
                         const cv::Mat& grey1,
                         cv::Mat& out,         // A float matrix
                         cv::Mat& confidence); // confidence only when apply_wls

   static void calculate(const Params& p,
                         const bool l_is_ref_image,
                         DisparityMethod method,
                         const cv::Mat& grey0,
                         const cv::Mat& grey1,
                         cv::Mat& out,         // A float matrix
                         cv::Mat& confidence); // confidence only when apply_wls

   friend string str(Disparity::DisparityMethod x) noexcept;
};

// string str(Disparity::DisparityMethod x) noexcept;
Disparity::DisparityMethod
to_disparity_method(const string_view) noexcept(false);

Disparity::DisparityMethod to_disparity_method(const int val) noexcept;

} // namespace perceive
