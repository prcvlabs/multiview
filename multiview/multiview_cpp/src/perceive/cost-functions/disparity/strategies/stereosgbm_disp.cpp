
#include "stereosgbm_disp.hpp"

#include "perceive/foundation.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#ifdef WITH_CUDA
#include <opencv2/cudastereo.hpp>
#endif

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#define This StereoSGBMDisparity

namespace perceive::disparity
{
constexpr float max_disparity = 150.0f;

// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams This::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, INT, sg_SAD_window_size, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_12_max_diff, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_min_disparity, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_num_disparities, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_speckle_window_size, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_speckle_range, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_prefilter_cap, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_uniqueness_ratio, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_mode, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_p1, false));
      m.push_back(MAKE_META(ThisParams, INT, sg_p2, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ------------------------------------------------------------ set ssgbm params
//
static void set_ssgbm_params(cv::Ptr<cv::StereoSGBM>& ssgbm_ptr,
                             const This::Params& p)
{
   auto sg_mode = p.sg_mode == cv::StereoSGBM::MODE_SGBM
                      ? p.sg_mode
                      : p.sg_mode == cv::StereoSGBM::MODE_HH
                            ? p.sg_mode
                            : p.sg_mode == cv::StereoSGBM::MODE_SGBM_3WAY
                                  ? p.sg_mode
                                  : p.sg_mode == cv::StereoSGBM::MODE_HH4
                                        ? p.sg_mode
                                        : cv::StereoSGBM::MODE_SGBM;

   // Stereo Algorithm
   ssgbm_ptr->setBlockSize(p.sg_SAD_window_size);             //
   ssgbm_ptr->setDisp12MaxDiff(p.sg_12_max_diff);             //
   ssgbm_ptr->setMinDisparity(p.sg_min_disparity);            //
   ssgbm_ptr->setNumDisparities(p.sg_num_disparities);        //
   ssgbm_ptr->setSpeckleRange(p.sg_speckle_range);            //
   ssgbm_ptr->setSpeckleWindowSize(p.sg_speckle_window_size); //
   ssgbm_ptr->setMode(sg_mode);
   ssgbm_ptr->setP1(p.sg_p1);
   ssgbm_ptr->setP2(p.sg_p2);
   ssgbm_ptr->setPreFilterCap(p.sg_prefilter_cap);       //
   ssgbm_ptr->setUniquenessRatio(p.sg_uniqueness_ratio); //
}

// ------------------------------------------------------------------- calculate
//
void This::calculate(const Params& p,
                     const bool l_is_ref_image,
                     const cv::Mat& image0,
                     const cv::Mat& image1,
                     cv::Mat& out_left) noexcept // A float matrix
{
   auto is_grey = [&](const cv::Mat& im) {
      return im.type() == CV_8U || im.type() == CV_8UC1;
   };

   if(image0.empty() or image1.empty())
      FATAL(format("attempt to calculate disparity on empty image"));
   if(!is_grey(image0) or !is_grey(image1)) {
      LOG_ERR(format("expected a pair of grey images"));
      return; // outta here!
   }

   cv::Mat out_right;

   struct TLParams
   {
      cv::Mat disp_16S;
   };
   static thread_local TLParams tl;

   const auto w = image0.cols;
   const auto h = image0.rows;

   auto ensure_dimensions = [w, h](cv::Mat& m, const int type) {
      if(m.rows != h || m.cols != w || m.type() != type)
         m = cv::Mat(h, w, type);
   };

   ensure_dimensions(out_left, CV_32FC1);
   ensure_dimensions(tl.disp_16S, CV_16S);

   try {
      cv::Ptr<cv::StereoSGBM> ssgbm_ptr = cv::StereoSGBM::create();
      set_ssgbm_params(ssgbm_ptr, p);

      if(l_is_ref_image) {
         ssgbm_ptr->compute(image0, image1, tl.disp_16S);
      } else {
         cv::Ptr<cv::StereoMatcher> ssgbmr_ptr
             = cv::ximgproc::createRightMatcher(ssgbm_ptr);
         ssgbmr_ptr->compute(image1, image0, tl.disp_16S);
      }

   } catch(std::exception& e) {
      LOG_ERR(format("Exception: {}", e.what()));
      return;
   } catch(...) {
      LOG_ERR("Unknown exception executing stereo-bm");
      return;
   }

   // -- Write results to float image
   if(CV_16S == tl.disp_16S.type()) {
      for(int y = 0; y < h; ++y) {
         const int16_t* src_ptr
             = reinterpret_cast<int16_t*>(tl.disp_16S.ptr(y));
         float* dst_ptr = reinterpret_cast<float*>(out_left.ptr(y));
         for(int x = 0; x < w; ++x) {
            auto val   = std::fabs(float(*src_ptr++)) / 16.0f;
            *dst_ptr++ = val > max_disparity ? 0.0f : val;
         }
      }
   } else if(CV_8U == tl.disp_16S.type()) {
      tl.disp_16S.convertTo(out_left, CV_32F);
   } else {
      FATAL("Invalid StereoBM result type");
   }
}

} // namespace perceive::disparity
