
#include "remap.hpp"

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

static void check_cuda(cudaError_t err, const char* what)
{
   if(err != cudaSuccess)
      throw std::runtime_error(format("{}: {}", what, cudaGetErrorString(err)));
}

// ------------------------------------------------------------------ RemapMaps

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

void RemapMaps::release_() noexcept
{
   cudaFree(mapx_);
   cudaFree(mapy_);
   mapx_   = nullptr;
   mapy_   = nullptr;
   width_  = 0;
   height_ = 0;
}

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

// ------------------------------------------------------------------- Remapper

Remapper::DeviceBuffer::~DeviceBuffer() { cudaFree(data); }

void Remapper::DeviceBuffer::create(size_t n_bytes)
{
   if(n_bytes <= size) return;
   cudaFree(data);
   data = nullptr;
   size = 0;
   check_cuda(cudaMalloc(&data, n_bytes), "cudaMalloc");
   size = n_bytes;
}

Remapper::~Remapper()
{
   if(stream_ != nullptr) cudaStreamDestroy(cudaStream_t(stream_));
}

bool Remapper::supports(const cv::Mat& src, int interpolation) noexcept
{
   return src.depth() == CV_8U and src.channels() <= 4
          and (interpolation == cv::INTER_NEAREST
               or interpolation == cv::INTER_LINEAR);
}

cv::Mat Remapper::remap(const cv::Mat& src,
                        const RemapMaps& maps,
                        int interpolation,
                        const cv::Scalar& border_value) const
{
   Expects(!src.empty());
   Expects(!maps.empty());
   Expects(supports(src, interpolation));

   lock_guard<std::mutex> lock(padlock_);

   if(stream_ == nullptr) {
      cudaStream_t s = nullptr;
      check_cuda(cudaStreamCreate(&s), "cudaStreamCreate");
      stream_ = s;
   }
   const auto stream = cudaStream_t(stream_);

   cv::Mat dst(int(maps.height()), int(maps.width()), src.type());
   const size_t src_row_bytes = src.elemSize() * size_t(src.cols);
   const size_t dst_row_bytes = dst.elemSize() * size_t(dst.cols);
   src_buffer_.create(src_row_bytes * size_t(src.rows));
   dst_buffer_.create(dst_row_bytes * size_t(dst.rows));

   check_cuda(cudaMemcpy2DAsync(src_buffer_.data,
                                src_row_bytes,
                                src.data,
                                src.step,
                                src_row_bytes,
                                size_t(src.rows),
                                cudaMemcpyHostToDevice,
                                stream),
              "cudaMemcpy2DAsync");

   const float border[4] = {float(border_value[0]),
                            float(border_value[1]),
                            float(border_value[2]),
                            float(border_value[3])};
   cuda_remap(static_cast<const uint8_t*>(src_buffer_.data),
              unsigned(src.cols),
              unsigned(src.rows),
              unsigned(src_row_bytes),
              maps.mapx(),
              maps.mapy(),
              static_cast<uint8_t*>(dst_buffer_.data),
              unsigned(dst.cols),
              unsigned(dst.rows),
              unsigned(dst_row_bytes),
              src.channels(),
              interpolation == cv::INTER_LINEAR,
              border,
              stream);
   check_cuda(cudaGetLastError(), "remap kernel");

   check_cuda(cudaMemcpyAsync(dst.data,
                              dst_buffer_.data,
                              dst_row_bytes * size_t(dst.rows),
                              cudaMemcpyDeviceToHost,
                              stream),
              "cudaMemcpyAsync");
   check_cuda(cudaStreamSynchronize(stream), "cudaStreamSynchronize");

   return dst;
}

} // namespace perceive::cuda
