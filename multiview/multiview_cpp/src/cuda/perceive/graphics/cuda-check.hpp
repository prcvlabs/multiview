
#pragma once

#include <cuda_runtime.h>
#include <sstream>
#include <stdexcept>

#define CUDA_CHECK(expr) \
   perceive::cuda::cuda_check_impl(expr, #expr, __FILE__, __LINE__)

namespace perceive
{
namespace cuda
{
   inline void cuda_check_impl(const cudaError_t err,
                               const char* expr,
                               const char* file,
                               unsigned line)
   {
      if(err != cudaSuccess) {
         std::stringstream ss;
         ss << "Error " << cudaGetErrorName(err) << " in " << file << ":"
            << line << " : " << expr;
         throw std::runtime_error(ss.str());
      }
   }

} // namespace cuda
} // namespace perceive
