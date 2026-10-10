
#include "remap.hpp"

#include "cuda-check.hpp"
#include "device-buffer.hpp"
#include "perceive/utils/cuda-spec.hpp"

#include <opencv2/imgproc.hpp>

namespace perceive::cuda
{
void cuda_remap(const uint8_t* src,
                unsigned src_width,
                unsigned src_height,
                const float* mapx,
                const float* mapy,
                uint8_t* dst,
                unsigned dst_width,
                unsigned dst_height,
                int n_channels,
                bool bilinear,
                const uint8_t border_value[4],
                cudaStream_t stream);

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

void RemapMaps::release_()
{
   if(mapx_ != nullptr) CUDA_CHECK(cudaFree(mapx_));
   if(mapy_ != nullptr) CUDA_CHECK(cudaFree(mapy_));
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

   const size_t row_bytes = sizeof(float) * size_t(mapx.cols);
   const size_t n_bytes   = row_bytes * size_t(mapx.rows);

   auto upload_one = [&](const cv::Mat& map, float*& dst) {
      CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&dst), n_bytes));
      CUDA_CHECK(cudaMemcpy2D(dst,
                              row_bytes,
                              map.data,
                              map.step,
                              row_bytes,
                              size_t(map.rows),
                              cudaMemcpyHostToDevice));
   };
   RemapMaps maps; // `*this` is unchanged if the upload fails
   upload_one(mapx, maps.mapx_);
   upload_one(mapy, maps.mapy_);
   maps.width_  = unsigned(mapx.cols);
   maps.height_ = unsigned(mapx.rows);

   *this = std::move(maps);
}

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

Remapper::Pimpl::Pimpl() { CUDA_CHECK(cudaStreamCreate(&stream)); }

Remapper::Pimpl::~Pimpl() { CUDA_CHECK(cudaStreamDestroy(stream)); }

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

   CUDA_CHECK(cudaMemcpy2DAsync(P.src_buffer.data(),
                                src_row_bytes,
                                src.data,
                                src.step,
                                src_row_bytes,
                                size_t(src.rows),
                                cudaMemcpyHostToDevice,
                                P.stream));

   const uint8_t border[4] = {cv::saturate_cast<uint8_t>(border_value[0]),
                              cv::saturate_cast<uint8_t>(border_value[1]),
                              cv::saturate_cast<uint8_t>(border_value[2]),
                              cv::saturate_cast<uint8_t>(border_value[3])};
   cuda_remap(static_cast<const uint8_t*>(P.src_buffer.data()),
              unsigned(src.cols),
              unsigned(src.rows),
              maps.mapx(),
              maps.mapy(),
              static_cast<uint8_t*>(P.dst_buffer.data()),
              unsigned(dst.cols),
              unsigned(dst.rows),
              src.channels(),
              interpolation == cv::INTER_LINEAR,
              border,
              P.stream);
   CUDA_CHECK(cudaGetLastError());

   CUDA_CHECK(cudaMemcpyAsync(dst.data,
                              P.dst_buffer.data(),
                              dst_row_bytes * size_t(dst.rows),
                              cudaMemcpyDeviceToHost,
                              P.stream));
   CUDA_CHECK(cudaStreamSynchronize(P.stream));

   return dst;
}

} // namespace perceive::cuda
