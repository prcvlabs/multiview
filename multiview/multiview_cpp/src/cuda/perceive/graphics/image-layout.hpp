
#pragma once

#include <cstddef>
#include <cuda_runtime.h>

namespace perceive
{
namespace cuda
{
   template<typename T> struct ImageLayout
   {
    public:
      unsigned width{0};
      unsigned height{0};
      T* pixels{nullptr};

      constexpr static size_t alloc_objects(unsigned width, unsigned height)
      {
         return size_t{width} * size_t{height};
      }
      constexpr static size_t alloc_bytes(unsigned width, unsigned height)
      {
         return sizeof(T) * alloc_objects(width, height);
      }
      __device__ __host__ unsigned pix_index(unsigned x, unsigned y) const
      {
         return width * y + x;
      }
      __device__ __host__ const T& pixel(const unsigned x,
                                         const unsigned y) const
      {
         return pixels[pix_index(x, y)];
      }
      __device__ __host__ T& pixel(const unsigned x, const unsigned y)
      {
         return pixels[pix_index(x, y)];
      }

      ImageLayout(const unsigned w, const unsigned h, T* pixels)
          : width(w)
          , height(h)
          , pixels(pixels)
      {}
      ImageLayout() = default;
   };

} // namespace cuda
} // namespace perceive
