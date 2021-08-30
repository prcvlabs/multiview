
#include "triclops_disp.hpp"

#include "perceive/foundation.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

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

#define This TriclopsDisparity

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
      m.push_back(MAKE_META(ThisParams, BOOL, rectify, true));
      m.push_back(MAKE_META(ThisParams, BOOL, lowpass, true));
      m.push_back(MAKE_META(ThisParams, BOOL, subpixel, true));
      m.push_back(MAKE_META(ThisParams, BOOL, subpixel_validation, true));
      m.push_back(MAKE_META(ThisParams, BOOL, edge_correlation, true));
      m.push_back(MAKE_META(ThisParams, INT, min_disp, true));
      m.push_back(MAKE_META(ThisParams, INT, max_disp, true));
      m.push_back(MAKE_META(ThisParams, INT, edge_mask, true));
      m.push_back(MAKE_META(ThisParams, INT, stereo_mask, true));
      m.push_back(MAKE_META(ThisParams, BOOL, texture_validation, true));
      m.push_back(MAKE_META(ThisParams, BOOL, surface_validation, true));
      m.push_back(MAKE_META(ThisParams, INT, surface_validation_size, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
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
      bool first{true};
      Params last_p;
#ifdef USE_TRICLOPS
      TriclopsContext triclops;
      TriclopsInput triclops_input;
      TriclopsError err;
      TriclopsImage im_disparity;
      TriclopsImage16 im_disp16;
#endif
   };
   static thread_local TLParams tl;

   if(tl.first) {
#ifdef USE_TRICLOPS
      tl.first = false;

      // Write triclops file
      string filename = format("/tmp/.bumblebee2.{}.{}.config", rand(), rand());
      FILE* fp        = fopen(filename.c_str(), "w");
      if(fp == NULL) FATAL(format("Could not open '{}' for writing", filename));
      fprintf(fp, "%s\n", triclops_bumblebee2_config_string().c_str());
      fclose(fp);

      // Init triclops context
      char* buffer = (char*) alloca(filename.size() + 1);
      strcpy(buffer, filename.c_str());
      tricheck(triclopsGetDefaultContextFromFile(&(tl.triclops), buffer),
               "Read triclops configuration file");

      // Remove temporary file
      unlink(filename.c_str());
#endif
   }

   const auto w = image0.cols;
   const auto h = image0.rows;

   auto ensure_dimensions = [w, h](cv::Mat& m, const int type) {
      if(m.rows != h || m.cols != w || m.type() != type)
         m = cv::Mat(h, w, type);
   };

   ensure_dimensions(out_left, CV_32FC1);

#ifdef USE_TRICLOPS
   auto stereo_triclops_calculate = [&]() {
      tricheck(triclopsSetResolutionAndPrepare(tl.triclops, h, w, h, w),
               "Set resolution");

      tricheck(triclopsSetRectify(tl.triclops, p.rectify), "Set rectify");
      tricheck(triclopsSetLowpass(tl.triclops, p.lowpass), "Set lowpass");
      tricheck(triclopsSetSubpixelInterpolation(tl.triclops, p.subpixel),
               "Set sub-pixel");
      tricheck(triclopsSetStrictSubpixelValidation(tl.triclops,
                                                   p.subpixel_validation),
               "Set strict sub-pixel validation");
      tricheck(triclopsSetEdgeCorrelation(tl.triclops, p.edge_correlation),
               "Set edge-correlation");
      tricheck(triclopsSetDisparity(tl.triclops, p.min_disp, p.max_disp),
               "Set disparity");
      tricheck(triclopsSetEdgeMask(tl.triclops, p.edge_mask), "Set edge-mask");
      tricheck(triclopsSetStereoMask(tl.triclops, p.stereo_mask),
               "Set stereo-mask");
      tricheck(triclopsSetTextureValidation(tl.triclops, p.texture_validation),
               "Set texture-validation");
      tricheck(triclopsSetSurfaceValidation(tl.triclops, p.surface_validation),
               "Set surface-validation");
      tricheck(triclopsSetSurfaceValidationSize(tl.triclops,
                                                p.surface_validation_size),
               "Set surface-validation-size");
      tricheck(triclopsSetDisparityMappingOn(tl.triclops, false),
               "Set disparity-mapping on");

      // Setup triclops input...
      unsigned long seconds       = 0;
      unsigned long micro_seconds = 0;

      auto ptr0 = (unsigned char*) image0.ptr<unsigned char>(0);
      auto ptr1 = (unsigned char*) image1.ptr<unsigned char>(0);
      tricheck(triclopsBuildRGBTriclopsInput(w,
                                             h,
                                             w,
                                             seconds,
                                             micro_seconds,
                                             ptr1,
                                             ptr0,
                                             ptr0,
                                             &tl.triclops_input),
               "triclopsBuildRGBTriclopsInput()");

      // Preprocess and Stereo
      tricheck(triclopsPreprocess(tl.triclops, &tl.triclops_input),
               "Preprocess images");

      // Generate disparity information
      tricheck(triclopsStereo(tl.triclops), "Stereo processing");

      TriclopsBool on;
      tricheck(triclopsGetSubpixelInterpolation(tl.triclops, &on),
               "Get subpixel interpolation");
      const bool is_subpixel = on;

      if(is_subpixel) {
         tricheck(triclopsGetImage16(tl.triclops,
                                     TriImg16_DISPARITY,
                                     TriCam_REFERENCE,
                                     &tl.im_disp16),
                  "Get disparity 16");

         // -- Write results to float image
         for(int y = 0; y < h; ++y) {
            const uint16_t* src_ptr = tl.im_disp16.data + y * w;
            float* dst_ptr          = (float*) out_left.ptr(y);
            for(int x = 0; x < w; ++x) {
               auto val   = float(*src_ptr++) / 256.0f;
               *dst_ptr++ = val > max_disparity ? 0.0f : val;
            }
         }
      } else {
         // Load disparity image directly
         tricheck(triclopsGetImage(tl.triclops,
                                   TriImg_DISPARITY,
                                   TriCam_REFERENCE,
                                   &tl.im_disparity),
                  "Load disparity image");

         // Set up floating-point disparity information
         for(int y = 0; y < h; ++y) {
            const uint8_t* src_ptr = tl.im_disparity.data + y * w;
            float* dst_ptr         = (float*) out_left.ptr(y);
            for(int x = 0; x < w; ++x) {
               auto val   = float(*src_ptr++);
               *dst_ptr++ = val > max_disparity ? 0.0f : val;
            }
         }
      }
   };
#else
   auto stereo_triclops_calculate = [&]() { LOG_ERR("Not compiled in"); };
#endif

   try {
      stereo_triclops_calculate();
   } catch(std::exception& e) {
      LOG_ERR(format("Exception: {}", e.what()));
      return;
   } catch(...) {
      LOG_ERR("Unknown exception executing stereo-bm");
      return;
   }
}

} // namespace perceive::disparity
