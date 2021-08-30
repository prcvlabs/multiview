
#include "sgm_disp.hpp"

#if defined WITH_CUDA && defined USE_SGM
#define SGM_IS_AVAILABLE
static constexpr bool k_use_sgm = true;
#else
static constexpr bool k_use_sgm = false;
#endif

#ifdef SGM_IS_AVAILABLE
#include <cuda.h>
#include <cuda_runtime.h>
#include <libsgm.h>
#endif

#include "perceive/foundation.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This SGMDisparity

namespace perceive::disparity
{
static constexpr float max_disparity = 512.0f;

// ---------------------------------------------------------------- DeviceBuffer
//
#ifdef SGM_IS_AVAILABLE
class DeviceBuffer
{
 private:
   void* data_ = nullptr;

 public:
   DeviceBuffer() = default;
   DeviceBuffer(size_t count) { allocate(count); }
   DeviceBuffer(const DeviceBuffer&) = delete;
   DeviceBuffer(DeviceBuffer&& o) noexcept { *this = std::move(o); }
   ~DeviceBuffer() { cudaFree(data_); }

   DeviceBuffer& operator=(const DeviceBuffer&) = delete;

   DeviceBuffer& operator=(DeviceBuffer&& o) noexcept
   {
      std::swap(data_, o.data_);
      return *this;
   }

   void allocate(size_t count) { cudaMalloc(&data_, count); }

   void* data() noexcept { return data_; }
};
#endif

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
   int ret = 128;
   switch(p.disparity_size) {
   case 64: [[fallthrough]];
   case 128: [[fallthrough]];
   case 256: ret = p.disparity_size; break;
   default:
      WARN(format("disparity-size = {}, but must be in {{}, {}, or "
                  "{}}. Using {}.",
                  p.disparity_size,
                  64,
                  128,
                  256,
                  ret));
   }
   return ret;
}

#ifdef SGM_IS_AVAILABLE
// -------------------------------------------------------------- calculate CUDA
//
static void calculate_(const This::Params& p,
                       const cv::Mat& image0,
                       const cv::Mat& image1,
                       cv::Mat& out_left) noexcept
{
   Expects(k_cuda_is_available == true);
   auto is_grey_8 = [](const cv::Mat& im) {
      return im.type() == CV_8U || im.type() == CV_8UC1;
   };

   auto is_grey_16 = [](const cv::Mat& im) {
      return im.type() == CV_16U || im.type() == CV_16UC1;
   };

   auto is_grey
       = [&](const cv::Mat& im) { return is_grey_8(im) || is_grey_16(im); };

   if(image0.empty() or image1.empty())
      FATAL(format("attempt to calculate disparity on empty image"));

   if(p.use_16bit_greyscale) {
      if(!is_grey_16(image0) || !is_grey_16(image1)) {
         LOG_ERR(format("expected a pair of 16-bit greyscale images"));
         return; // outta here!
      }
   } else { // expects grey 16
      if(!is_grey_8(image0) || !is_grey_8(image1)) {
         LOG_ERR(format("expected a pair of 8-bit greyscale images"));
         return; // outta here!
      }
   }

   struct TLParams
   {
      cv::Mat disparity;
   };
   static thread_local TLParams tl;

   const auto now = tick();

   const auto w = image0.cols;
   const auto h = image0.rows;

   const int disp_size = disparity_size(p);
   const int in_depth  = p.use_16bit_greyscale ? 16 : 8;
   const int in_bytes  = (in_depth * w * h) / 8;

   const bool output_16 = (disp_size == 256);
   const int out_depth  = (output_16 == true) ? 16 : 8;
   const int out_bytes  = (out_depth * w * h) / 8;

   sgm::StereoSGM sgm(
       w, h, disp_size, in_depth, out_depth, sgm::EXECUTE_INOUT_CUDA2CUDA);

   const int invalid_disp
       = (out_depth == 8) ? static_cast<uint8_t>(sgm.get_invalid_disparity())
                          : static_cast<uint16_t>(sgm.get_invalid_disparity());

   auto& disparity = tl.disparity;

   auto ensure_dimensions = [w, h](cv::Mat& m, const int type) {
      if(m.rows != h || m.cols != w || m.type() != type)
         m = cv::Mat(h, w, type);
      Expects(w == m.cols);
      Expects(h == m.rows);
      Expects(type == m.type());
   };

   ensure_dimensions(disparity, (out_depth == 8 ? CV_8U : CV_16U));
   ensure_dimensions(out_left, CV_32FC1);

   DeviceBuffer in0(in_bytes), in1(in_bytes), outbuf(out_bytes);

   cudaMemcpy(in0.data(), image0.data, in_bytes, cudaMemcpyHostToDevice);
   cudaMemcpy(in1.data(), image1.data, in_bytes, cudaMemcpyHostToDevice);

   sgm.execute(in0.data(), in1.data(), outbuf.data());
   cudaDeviceSynchronize();

   if(false) {
      INFO(format("in-depth = {}, out-depth = {}, out-bytes = {}, wh = "
                  "[{}x{}], "
                  "disp = [{}x{}]",
                  in_depth,
                  out_depth,
                  out_bytes,
                  w,
                  h,
                  disparity.rows,
                  disparity.cols));
   }

   cudaMemcpy(disparity.data, outbuf.data(), out_bytes, cudaMemcpyDeviceToHost);

   if(disparity.type() == CV_16U) {
      for(auto y = 0; y < h; ++y) {
         const uint16_t* src_ptr
             = reinterpret_cast<uint16_t*>(disparity.ptr(y));
         float* dst_ptr = reinterpret_cast<float*>(out_left.ptr(y));
         for(auto x = 0; x < w; ++x) {
            auto val   = std::fabs(float(*src_ptr++));
            *dst_ptr++ = val > max_disparity ? 0.0f : val;
         }
      }
   } else if(disparity.type() == CV_8U) {
      disparity.convertTo(out_left, CV_32F);
   } else {
      FATAL("Invalid StereoBM result type");
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
