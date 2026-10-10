#pragma once

#include <mutex>
#include <opencv2/core.hpp>

namespace perceive::cuda
{
#ifdef WITH_CUDA

// A pair of CV_32FC1 remap tables (see cv::remap) in GPU memory, so that
// they are uploaded once and reused for every frame.
class RemapMaps
{
 private:
   float* mapx_     = nullptr;
   float* mapy_     = nullptr;
   unsigned width_  = 0;
   unsigned height_ = 0;

   void release_() noexcept;

 public:
   RemapMaps()                 = default;
   RemapMaps(const RemapMaps&) = delete;
   RemapMaps(RemapMaps&&) noexcept;
   ~RemapMaps();
   RemapMaps& operator=(const RemapMaps&) = delete;
   RemapMaps& operator=(RemapMaps&&) noexcept;

   bool empty() const noexcept { return mapx_ == nullptr; }
   unsigned width() const noexcept { return width_; }
   unsigned height() const noexcept { return height_; }
   const float* mapx() const noexcept { return mapx_; }
   const float* mapy() const noexcept { return mapy_; }

   void upload(const cv::Mat& mapx, const cv::Mat& mapy);
};

// cv::remap(..., cv::BORDER_CONSTANT, border_value) on the GPU, for 8-bit
// images with 1-4 channels, and nearest or bilinear interpolation.
class Remapper
{
 private:
   struct DeviceBuffer
   {
      void* data{nullptr};
      size_t size{0};
      DeviceBuffer()                               = default;
      DeviceBuffer(const DeviceBuffer&)            = delete;
      DeviceBuffer& operator=(const DeviceBuffer&) = delete;
      ~DeviceBuffer();
      void create(size_t n_bytes); // no-op if already large enough
   };

   mutable DeviceBuffer src_buffer_;
   mutable DeviceBuffer dst_buffer_;
   mutable void* stream_{nullptr}; // cudaStream_t, created lazily
   mutable std::mutex padlock_;

 public:
   Remapper()                           = default;
   Remapper(const Remapper&)            = delete;
   Remapper& operator=(const Remapper&) = delete;
   ~Remapper();

   static bool supports(const cv::Mat& src, int interpolation) noexcept;

   cv::Mat remap(const cv::Mat& src,
                 const RemapMaps& maps,
                 int interpolation,
                 const cv::Scalar& border_value) const;
};

#else

class RemapMaps
{
 public:
   bool empty() const noexcept { return true; }
   void upload(const cv::Mat& mapx, const cv::Mat& mapy) {}
};

class Remapper
{
 public:
   static bool supports(const cv::Mat& src, int interpolation) noexcept
   {
      return false;
   }

   cv::Mat remap(const cv::Mat& src,
                 const RemapMaps& maps,
                 int interpolation,
                 const cv::Scalar& border_value) const
   {
      FATAL("Cuda is not available!");
      return cv::Mat{};
   }
};

#endif

} // namespace perceive::cuda
