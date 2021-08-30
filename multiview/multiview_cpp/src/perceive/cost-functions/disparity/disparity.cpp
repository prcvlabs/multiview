
#include "perceive/foundation.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#include "disparity.hpp"

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

// -------------------------------------------------------------------- TRICLOPS
#ifdef USE_TRICLOPS

#include <unistd.h>

#include "perceive/contrib/triclops/triclops-bumblebee2-config-string.hpp"
#include <fc2triclops.h>
#include <triclops.h>

// aliases namespaces
namespace FC2  = FlyCapture2;
namespace FC2T = Fc2Triclops;

inline void tricheck(TriclopsError err, const std::string& msg)
{
   if(err != TriclopsErrorOk) {
      LOG_ERR(perceive::format("Triclops err code {}: {}", err, msg));
      throw std::runtime_error(msg);
   }
}

#endif
// ------------------------------------------------------------------- /TRICLOPS

#define This Disparity

namespace perceive
{
// ------------------------------------------------------------------- Calculate

void This::calculate(const Params& p,
                     const bool l_is_ref_image,
                     const cv::Mat& image0,
                     const cv::Mat& image1,
                     cv::Mat& out_left,
                     cv::Mat& confidence)
{
   calculate(p,
             l_is_ref_image,
             p.disparity_method,
             image0,
             image1,
             out_left,
             confidence);
}

void This::calculate(const Params& p,
                     const bool l_is_ref_image,
                     DisparityMethod method,
                     const cv::Mat& image0,
                     const cv::Mat& image1,
                     cv::Mat& out_left,
                     cv::Mat& confidence)
{
   if(image0.empty() or image1.empty())
      FATAL(format("attempt to calculate disparity on empty image"));

   struct TLParams
   {
      cv::Mat img_disp_8U;
      cv::Mat grey[2];
      cv::Mat grey_blur[2];
   };
   static thread_local TLParams tl;

   const bool grey_16
       = (method == CUDA_SGM) and (p.sgm_params.use_16bit_greyscale == true);
   const bool grey_8 = !grey_16;

   const auto w = image0.cols;
   const auto h = image0.rows;

   auto is_grey_8 = [](const cv::Mat& im) {
      return im.type() == CV_8U || im.type() == CV_8UC1;
   };

   auto is_grey_16 = [](const cv::Mat& im) {
      return im.type() == CV_16U || im.type() == CV_16UC1;
   };

   // We convert to greyscale, or reuse the input images if they are greyscale
   const cv::Mat* grey0_ptr{nullptr};
   const cv::Mat* grey1_ptr{nullptr};

   auto proc_grey
       = [&](const cv::Mat& im, cv::Mat& tl_im, decltype(grey0_ptr)& grey_ptr) {
            if(grey_8) {
               if(is_grey_8(im)) {
                  grey_ptr = &im;
               } else {
                  cv::cvtColor(im, tl_im, cv::COLOR_BGR2GRAY);
                  grey_ptr = &tl_im;
               }
               // INFO(format("grey-8 = {}", str(grey_8)));
               // cout << format("z = {} (cv8u = {}, cv8uc1 = {})",
               //                grey_ptr->type(),
               //                CV_8U,
               //                CV_8UC1)
               //      << endl;
               // FATAL("kBAM!");
            } else if(grey_16) {
               if(is_grey_16(im)) {
                  grey_ptr = &im;
               } else {
                  cv_to_cv_grey16(im, tl_im);
                  grey_ptr = &tl_im;
               }
            } else {
               FATAL("should be unreachable");
            }
         };

   auto set_grey_pointers = [&]() {
      proc_grey(image0, tl.grey[0], grey0_ptr);
      proc_grey(image1, tl.grey[1], grey1_ptr);

      for(auto i = 0; i < 2; ++i) {
         if(p.presmooth_sigma > 0.0) {
            auto sz = cv::Size(31, 31);
            cv::GaussianBlur(*grey0_ptr,
                             tl.grey_blur[0],
                             sz,
                             p.presmooth_sigma,
                             p.presmooth_sigma,
                             cv::BORDER_DEFAULT);
            cv::GaussianBlur(*grey1_ptr,
                             tl.grey_blur[1],
                             sz,
                             p.presmooth_sigma,
                             p.presmooth_sigma,
                             cv::BORDER_DEFAULT);
            grey0_ptr = &(tl.grey_blur[0]);
            grey1_ptr = &(tl.grey_blur[1]);
         }
      }
   };

   set_grey_pointers();

   auto s = time_thunk([&]() {
      switch(method) {
      case STEREO_BM:
         disparity::StereoBMDisparity::calculate(p.stereobm_params,
                                                 true,
                                                 p.use_cuda_if_available,
                                                 *grey0_ptr,
                                                 *grey1_ptr,
                                                 out_left);
         break;
      case STEREO_SGBM:
         disparity::StereoSGBMDisparity::calculate(
             p.stereosgbm_params, true, *grey0_ptr, *grey1_ptr, out_left);
         break;
      case TRICLOPS:
         disparity::TriclopsDisparity::calculate(
             p.triclops_params, true, *grey0_ptr, *grey1_ptr, out_left);
         break;
      case CUDA_SGM:
         disparity::SGMDisparity::calculate(
             p.sgm_params, *grey0_ptr, *grey1_ptr, out_left);
         break;
      default: FATAL("kBAM!");
      }
   });

   // INFO(format("method {} took {}s", str(method), s));
}

// ----------------------------------------------------------------- to/from str
//
string str(Disparity::DisparityMethod x) noexcept
{
   switch(x) {
   case Disparity::STEREO_BM: return "STEREO_BM"s;
   case Disparity::STEREO_SGBM: return "STEREO_SGBM"s;
   case Disparity::TRICLOPS: return "TRICLOPS"s;
   case Disparity::CUDA_SGM: return "CUDA_SGM"s;
   }
   Expects(false);
   return ""s;
}

Disparity::DisparityMethod
to_disparity_method(const string_view s) noexcept(false)
{
   if(s == "STEREO_BM"s) return Disparity::STEREO_BM;
   if(s == "STEREO_SGBM"s) return Disparity::STEREO_SGBM;
   if(s == "TRICLOPS"s) return Disparity::TRICLOPS;
   if(s == "CUDA_SGM"s) return Disparity::CUDA_SGM;
   throw std::runtime_error(format("bad disparity method value: '{}'", s));
}

Disparity::DisparityMethod to_disparity_method(const int val) noexcept
{
   const auto x = Disparity::DisparityMethod(val);
   switch(x) {
   case Disparity::STEREO_BM: return x;
   case Disparity::STEREO_SGBM: return x;
   case Disparity::TRICLOPS: return x;
   case Disparity::CUDA_SGM: return x;
   default: Expects(false);
   }
   return Disparity::STEREO_BM;
}

// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& Disparity::Params::meta_data() const noexcept
{
#define ThisParams Disparity::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, use_cuda_if_available, false));
      m.push_back({meta_type::STRING,
                   "disparity_method"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.disparity_method)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o            = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s    = std::any_cast<const string>(x);
                      o.disparity_method = to_disparity_method(s);
                   }});
      m.push_back(MAKE_META(ThisParams, REAL, presmooth_sigma, true));

      m.push_back(
          MAKE_META(ThisParams, COMPATIBLE_OBJECT, stereobm_params, true));
      m.push_back(
          MAKE_META(ThisParams, COMPATIBLE_OBJECT, stereosgbm_params, true));
      m.push_back(
          MAKE_META(ThisParams, COMPATIBLE_OBJECT, triclops_params, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, sgm_params, true));

      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

} // namespace perceive
