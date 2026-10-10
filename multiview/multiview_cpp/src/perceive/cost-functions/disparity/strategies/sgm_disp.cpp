
#include "sgm_disp.hpp"

#if defined WITH_CUDA && defined USE_SGM
#define SGM_IS_AVAILABLE
static constexpr bool k_use_sgm = true;
#else
static constexpr bool k_use_sgm = false;
#endif

#ifdef SGM_IS_AVAILABLE
#include <libsgm.h>
#endif

#include "perceive/foundation.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/threads.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This SGMDisparity

namespace perceive::disparity
{
// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams This::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, INT, disparity_size, true));
      m.push_back(MAKE_META(ThisParams, BOOL, use_16bit_greyscale, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// -------------------------------------------------------------- disparity-size
//
static int disparity_size(const This::Params& p) noexcept
{
   // contrib/libSGM supports 64 and 128 disparities
   int ret = 128;
   switch(p.disparity_size) {
   case 64: [[fallthrough]];
   case 128: ret = p.disparity_size; break;
   default:
      WARN(format("disparity-size = {}, but must be 64 or 128. Using {}.",
                  p.disparity_size,
                  ret));
   }
   return ret;
}

#ifdef SGM_IS_AVAILABLE
// ---------------------------------------------------------------- sgm-registry
//
// libSGM allocates its cost volumes up front (hundreds of megabytes), so one
// instance per configuration is shared by all threads. The GPU serializes
// the calls anyway.
struct SGMRegistry
{
 private:
   SGMRegistry()
   {
      register_exit_function([this]() { reset(); });
   }

   using Key = std::tuple<int, int, int, int>; // w, h, disp-size, in-depth

   std::mutex padlock_;
   std::map<Key, unique_ptr<sgm::StereoSGM>> registry_ = {};

 public:
   static SGMRegistry& instance() noexcept
   {
      static SGMRegistry registry;
      return registry;
   }

   void reset() noexcept
   {
      lock_guard lock(padlock_);
      registry_.clear();
   }

   /// @precondition: `in0` and `in1` are continuous, and the same size.
   void execute(const cv::Mat& in0,
                const cv::Mat& in1,
                const int disp_size,
                const int in_depth,
                cv::Mat& disparity)
   {
      const int w = in0.cols;
      const int h = in0.rows;

      lock_guard lock(padlock_);
      auto& engine = registry_[Key{w, h, disp_size, in_depth}];
      if(engine == nullptr)
         engine = make_unique<sgm::StereoSGM>(
             w, h, disp_size, in_depth, 16, sgm::EXECUTE_INOUT_HOST2HOST);

      disparity.create(h, w, CV_16UC1);
      void* dst = disparity.data;
      engine->execute(in0.data, in1.data, &dst);
   }
};

// -------------------------------------------------------------- calculate CUDA
//
// libSGM needs the width and height to be multiples of 32, and the width to
// be at least the disparity range. Inputs are padded (edge-replicated) on the
// right/bottom, and the result cropped back.
static void calculate_(const This::Params& p,
                       const cv::Mat& image0,
                       const cv::Mat& image1,
                       cv::Mat& out_left) noexcept
{
   Expects(k_cuda_is_available == true);
   if(!cuda::cuda_is_available()) { // libSGM exit()s on CUDA errors
      LOG_ERR(format("SGM stereo correspondence needs a CUDA device"));
      return;
   }
   if(image0.empty() or image1.empty())
      FATAL(format("attempt to calculate disparity on empty image"));

   const int in_type = p.use_16bit_greyscale ? CV_16UC1 : CV_8UC1;
   if(image0.type() != in_type || image1.type() != in_type) {
      LOG_ERR(format("expected a pair of {}-bit greyscale images",
                     p.use_16bit_greyscale ? 16 : 8));
      return; // outta here!
   }
   if(image0.size() != image1.size()) {
      LOG_ERR(format("SGM: left and right images differ in size"));
      return;
   }

   const int w         = image0.cols;
   const int h         = image0.rows;
   const int disp_size = disparity_size(p);
   const int in_depth  = p.use_16bit_greyscale ? 16 : 8;
   auto round_up       = [](int x, int m) { return ((x + m - 1) / m) * m; };
   const int pw        = round_up(std::max(w, disp_size), 32);
   const int ph        = round_up(h, 32);

   auto pad = [&](const cv::Mat& src) -> cv::Mat {
      if(pw == w and ph == h and src.isContinuous()) return src;
      cv::Mat dst;
      cv::copyMakeBorder(src, dst, 0, ph - h, 0, pw - w, cv::BORDER_REPLICATE);
      return dst;
   };

   cv::Mat disparity;
   try {
      SGMRegistry::instance().execute(
          pad(image0), pad(image1), disp_size, in_depth, disparity);
   } catch(std::exception& e) {
      LOG_ERR(format("libSGM failed: {}", e.what()));
      return;
   }

   // libSGM reports (disparity + 1), with 0 meaning invalid
   out_left.create(h, w, CV_32FC1);
   for(auto y = 0; y < h; ++y) {
      const uint16_t* src = disparity.ptr<uint16_t>(y);
      float* dst          = out_left.ptr<float>(y);
      for(auto x = 0; x < w; ++x)
         dst[x] = (src[x] == 0) ? 0.0f : float(src[x] - 1);
   }
}

#else
// ------------------------------------------------- calculate NO CUDA or NO SGM
//
static void calculate_(const This::Params& p,
                       const cv::Mat& image0,
                       const cv::Mat& image1,
                       cv::Mat& out_left) noexcept
{
   Expects(k_cuda_is_available == false or k_use_sgm == false);

   if(k_cuda_is_available and k_use_sgm) {
      Expects(false);
   } else if(k_cuda_is_available and !k_use_sgm) {
      LOG_ERR(format("SGM stereo correspondence not availble because, even "
                     "though CUDA is available, libSGM was not compiled in."));
   } else if(!k_cuda_is_available) {
      LOG_ERR(format("SGM stereo correspondence not availble because CUDA is "
                     "not available."));
   } else {
      Expects(false);
   }

   return; // outta here!
}
#endif

// ------------------------------------------------------------------- calculate
//
void This::calculate(const Params& p,
                     const cv::Mat& image0,
                     const cv::Mat& image1,
                     cv::Mat& out_left) noexcept // A float matrix
{
   calculate_(p, image0, image1, out_left);
}

} // namespace perceive::disparity
