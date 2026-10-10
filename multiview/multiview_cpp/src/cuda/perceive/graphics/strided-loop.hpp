
#pragma once

namespace perceive
{
namespace cuda
{
   // Grid-stride loops: each thread calls `body` for every index it owns,
   // so any launch configuration covers the whole range.

   template<typename F>
   __device__ inline void strided_loop_1d(const unsigned width, const F& body)
   {
      const unsigned chunk_width{blockDim.x * gridDim.x};
      const unsigned chunk_x{blockIdx.x * blockDim.x + threadIdx.x};
      for(unsigned x{chunk_x}; x < width; x += chunk_width) { body(x); }
   }

   template<typename F>
   __device__ inline void
   strided_loop_2d(const unsigned width, const unsigned height, const F& body)
   {
      const unsigned chunk_width{blockDim.x * gridDim.x};
      const unsigned chunk_height{blockDim.y * gridDim.y};
      const unsigned chunk_x{blockIdx.x * blockDim.x + threadIdx.x};
      const unsigned chunk_y{blockIdx.y * blockDim.y + threadIdx.y};

      for(unsigned y{chunk_y}; y < height; y += chunk_height) {
         for(unsigned x{chunk_x}; x < width; x += chunk_width) { body(x, y); }
      }
   }

} // namespace cuda
} // namespace perceive
