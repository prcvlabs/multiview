
#include "stereobm_disp.hpp"

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

#define This StereoBMDisparity

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
      m.push_back(MAKE_META(ThisParams, INT, bm_SAD_window_size, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_12_max_diff, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_min_disparity, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_num_disparities, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_speckle_window_size, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_speckle_range, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_prefilter_cap, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_prefilter_size, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_prefilter_type, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_block_size, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_texture_threshold, true));
      m.push_back(MAKE_META(ThisParams, INT, bm_uniqueness_ratio, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

static bool use_cuda(const bool use_cuda_if_available) noexcept
{
   return k_cuda_is_available and use_cuda_if_available;
}

static void set_sbm_params(cv::Ptr<cv::StereoBM>& sbm_ptr,
                           const This::Params& p)
{
   auto prefilt_type = (p.bm_prefilter_type == cv::StereoBM::PREFILTER_XSOBEL)
                           ? cv::StereoBM::PREFILTER_XSOBEL
                           : cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;

   // Stereo Algorithm
   sbm_ptr->setBlockSize(p.bm_SAD_window_size);             //
   sbm_ptr->setDisp12MaxDiff(p.bm_12_max_diff);             //
   sbm_ptr->setMinDisparity(p.bm_min_disparity);            //
   sbm_ptr->setNumDisparities(p.bm_num_disparities);        //
   sbm_ptr->setSpeckleRange(p.bm_speckle_range);            //
   sbm_ptr->setSpeckleWindowSize(p.bm_speckle_window_size); //
   sbm_ptr->setPreFilterCap(p.bm_prefilter_cap);            //
   sbm_ptr->setPreFilterSize(p.bm_prefilter_size);          //
   sbm_ptr->setPreFilterType(prefilt_type);                 //
   sbm_ptr->setSmallerBlockSize(p.bm_block_size);           //
   sbm_ptr->setTextureThreshold(p.bm_texture_threshold);    //
   sbm_ptr->setUniquenessRatio(p.bm_uniqueness_ratio);      //
}

// ------------------------------------------------------------------- calculate
//
void This::calculate(const Params& p,
                     const bool l_is_ref_image,
                     const bool use_cuda_if_available,
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
      Params last_p;
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

   auto run_stereo_bm_cuda
       = [&](const cv::Mat& left, const cv::Mat& right, cv::Mat& result) {
#ifdef WITH_CUDA
            cv::Ptr<cv::StereoBM> sbm_ptr = cv::cuda::createStereoBM();
            set_sbm_params(sbm_ptr, p);
            cv::cuda::GpuMat gpu_left{left};
            cv::cuda::GpuMat gpu_right{right};
            cv::cuda::GpuMat gpu_result;

            // cuda::set_device(1);
            // cuda::print_cuda_report();

            const auto now = tick();
            if(!l_is_ref_image) {
               auto sbm_right = cv::ximgproc::createRightMatcher(sbm_ptr);
               sbm_right->compute(gpu_left, gpu_right, gpu_result);
            } else {
               sbm_ptr->compute(gpu_left, gpu_right, gpu_result);
            }
            gpu_result.download(result);
#else
            FATAL("attempt to run cuda version, but not compiled with cuda");
#endif
         };

   auto run_stereo_bm_cpu
       = [&](const cv::Mat& left, const cv::Mat& right, cv::Mat& result) {
            // INFO("Running cpu StereoBM");
            cv::Ptr<cv::StereoBM> sbm_ptr = cv::StereoBM::create();
            set_sbm_params(sbm_ptr, p);
            if(!l_is_ref_image) {
               auto sbm_right = cv::ximgproc::createRightMatcher(sbm_ptr);
               sbm_right->compute(left, right, result);
            } else {
               sbm_ptr->compute(left, right, result);
            }
         };

   auto run_stereo_bm
       = [&](const cv::Mat& left, const cv::Mat& right, cv::Mat& result) {
            if(use_cuda(use_cuda_if_available)) {
               if(true) {
                  WARN(format(
                      "cuda-stereobm is broken, running cpu-stereobm anyway."));
                  run_stereo_bm_cpu(left, right, result);
               } else {
                  run_stereo_bm_cuda(left, right, result);
               }
            } else {
               run_stereo_bm_cpu(left, right, result);
            }
         };

   try {
      run_stereo_bm(image0, image1, tl.disp_16S);
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
