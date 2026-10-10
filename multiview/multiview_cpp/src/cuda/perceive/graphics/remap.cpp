
#include "remap.hpp"

#include "perceive/utils/cuda-spec.hpp"

#include <cuda_runtime.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace perceive::cuda
{
void cuda_remap(const uint8_t* src,
                unsigned src_width,
                unsigned src_height,
                unsigned src_row_bytes,
                const float* mapx,
                const float* mapy,
                uint8_t* dst,
                unsigned dst_width,
                unsigned dst_height,
                unsigned dst_row_bytes,
                int n_channels,
                bool bilinear,
                const float border_value[4],
                cudaStream_t stream);

// ------------------------------------------------------------------ check-cuda

static void check_cuda(cudaError_t err, const char* what)
{
   if(err != cudaSuccess)
      throw std::runtime_error(format("{}: {}", what, cudaGetErrorString(err)));
}

// ------------------------------------------------------ RemapMaps construction

RemapMaps::RemapMaps(RemapMaps&& o) noexcept { *this = std::move(o); }

RemapMaps::~RemapMaps() { release_(); }

RemapMaps& RemapMaps::operator=(RemapMaps&& o) noexcept
{
   std::swap(mapx_, o.mapx_);
   std::swap(mapy_, o.mapy_);
   std::swap(width_, o.width_);
   std::swap(height_, o.height_);
   return *this;
}

// ---------------------------------------------------------- RemapMaps::release

void RemapMaps::release_() noexcept
{
   cudaFree(mapx_);
   cudaFree(mapy_);
   mapx_   = nullptr;
   mapy_   = nullptr;
   width_  = 0;
   height_ = 0;
}

// ----------------------------------------------------------- RemapMaps::upload

void RemapMaps::upload(const cv::Mat& mapx, const cv::Mat& mapy)
{
   Expects(mapx.type() == CV_32FC1 and mapy.type() == CV_32FC1);
   Expects(mapx.size() == mapy.size());

   release_();
   const size_t row_bytes = sizeof(float) * size_t(mapx.cols);
   const size_t n_bytes   = row_bytes * size_t(mapx.rows);

   auto upload_one = [&](const cv::Mat& map, float*& dst) {
      check_cuda(cudaMalloc(reinterpret_cast<void**>(&dst), n_bytes),
                 "cudaMalloc");
      check_cuda(cudaMemcpy2D(dst,
                              row_bytes,
                              map.data,
                              map.step,
                              row_bytes,
                              size_t(map.rows),
                              cudaMemcpyHostToDevice),
                 "cudaMemcpy2D");
   };
   upload_one(mapx, mapx_);
   upload_one(mapy, mapy_);
   width_  = unsigned(mapx.cols);
   height_ = unsigned(mapx.rows);
}

// ---------------------------------------------------------------- DeviceBuffer

struct DeviceBuffer
{
   void* data{nullptr};
   size_t size{0};
   DeviceBuffer()                               = default;
   DeviceBuffer(const DeviceBuffer&)            = delete;
   DeviceBuffer& operator=(const DeviceBuffer&) = delete;
   ~DeviceBuffer() { cudaFree(data); }

   void create(size_t n_bytes) // no-op if already large enough
   {
      if(n_bytes <= size) return;
      cudaFree(data);
      data = nullptr;
      size = 0;
      check_cuda(cudaMalloc(&data, n_bytes), "cudaMalloc");
      size = n_bytes;
   }
};

// ------------------------------------------------------------- Remapper::Pimpl

struct Remapper::Pimpl
{
   DeviceBuffer src_buffer;
   DeviceBuffer dst_buffer;
   cudaStream_t stream{nullptr};
   std::mutex lock;

   Pimpl();
   ~Pimpl();
};

Remapper::Pimpl::Pimpl()
{
   check_cuda(cudaStreamCreate(&stream), "cudaStreamCreate");
}

Remapper::Pimpl::~Pimpl() { cudaStreamDestroy(stream); }

// ------------------------------------------------------- Remapper construction

Remapper::Remapper()
{
   if(cuda_is_available()) pimpl_ = std::make_unique<Pimpl>();
}

Remapper::~Remapper() = default;

// ---------------------------------------------------------- Remapper::supports

bool Remapper::supports(const cv::Mat& src, int interpolation) noexcept
{
   return src.depth() == CV_8U and src.channels() <= 4
          and (interpolation == cv::INTER_NEAREST
               or interpolation == cv::INTER_LINEAR);
}

// ------------------------------------------------------------- Remapper::remap

cv::Mat Remapper::remap(const cv::Mat& src,
                        const RemapMaps& maps,
                        int interpolation,
                        const cv::Scalar& border_value) const
{
   Expects(!src.empty());
   Expects(!maps.empty());
   Expects(supports(src, interpolation));
   Expects(pimpl_ != nullptr);

   auto& P = *pimpl_;
   lock_guard<std::mutex> padlock(P.lock);

   cv::Mat dst(int(maps.height()), int(maps.width()), src.type());
   const size_t src_row_bytes = src.elemSize() * size_t(src.cols);
   const size_t dst_row_bytes = dst.elemSize() * size_t(dst.cols);
   P.src_buffer.create(src_row_bytes * size_t(src.rows));
   P.dst_buffer.create(dst_row_bytes * size_t(dst.rows));

   check_cuda(cudaMemcpy2DAsync(P.src_buffer.data,
                                src_row_bytes,
                                src.data,
                                src.step,
                                src_row_bytes,
                                size_t(src.rows),
                                cudaMemcpyHostToDevice,
                                P.stream),
              "cudaMemcpy2DAsync");

   const float border[4] = {float(border_value[0]),
                            float(border_value[1]),
                            float(border_value[2]),
                            float(border_value[3])};
   cuda_remap(static_cast<const uint8_t*>(P.src_buffer.data),
              unsigned(src.cols),
              unsigned(src.rows),
              unsigned(src_row_bytes),
              maps.mapx(),
              maps.mapy(),
              static_cast<uint8_t*>(P.dst_buffer.data),
              unsigned(dst.cols),
              unsigned(dst.rows),
              unsigned(dst_row_bytes),
              src.channels(),
              interpolation == cv::INTER_LINEAR,
              border,
              P.stream);
   check_cuda(cudaGetLastError(), "remap kernel");

   check_cuda(cudaMemcpyAsync(dst.data,
                              P.dst_buffer.data,
                              dst_row_bytes * size_t(dst.rows),
                              cudaMemcpyDeviceToHost,
                              P.stream),
              "cudaMemcpyAsync");
   check_cuda(cudaStreamSynchronize(P.stream), "cudaStreamSynchronize");

   return dst;
}

} // namespace perceive::cuda
