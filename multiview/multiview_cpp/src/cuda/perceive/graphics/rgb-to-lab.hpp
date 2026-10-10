
#pragma once

#include "perceive/graphics/image-container.hpp"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <type_traits>

namespace perceive::cuda
{
#ifdef WITH_CUDA

class ARGBToLABConversion
{
   // Zero-copy (mapped, pinned) host memory that the kernel reads/writes
   // directly.
   struct MappedBuffer
   {
      void* data{nullptr};
      size_t size{0};
      MappedBuffer()                               = default;
      MappedBuffer(const MappedBuffer&)            = delete;
      MappedBuffer& operator=(const MappedBuffer&) = delete;
      ~MappedBuffer();
      void create(size_t n_bytes); // no-op if already large enough
   };

   mutable MappedBuffer input_buffer;
   mutable MappedBuffer output_buffer;
   mutable void* stream{nullptr}; // cudaStream_t, created lazily
   mutable std::mutex lock;

 public:
   ARGBToLABConversion()                                      = default;
   ARGBToLABConversion(const ARGBToLABConversion&)            = delete;
   ARGBToLABConversion& operator=(const ARGBToLABConversion&) = delete;
   ~ARGBToLABConversion();

   Vec3fImage convert(const ARGBImage& argb) const;
};

#else

class ARGBToLABConversion
{
 public:
   Vec3fImage convert(const ARGBImage& argb) const
   {
      FATAL("Cuda is not available!");
      return Vec3fImage{};
   }
};

#endif

} // namespace perceive::cuda
