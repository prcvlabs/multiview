
#pragma once

#include "cuda-check.hpp"

#include <utility>

namespace perceive
{
namespace cuda
{
   class DeviceBuffer
   {
    private:
      void* data_  = nullptr;
      size_t size_ = 0;

    public:
      DeviceBuffer() = default;
      DeviceBuffer(size_t n_bytes) { create(n_bytes); }
      DeviceBuffer(const DeviceBuffer&) = delete;
      DeviceBuffer(DeviceBuffer&& o) noexcept { *this = std::move(o); }
      ~DeviceBuffer()
      {
         if(data_ != nullptr) CUDA_CHECK(cudaFree(data_));
      }

      DeviceBuffer& operator=(const DeviceBuffer&) = delete;

      DeviceBuffer& operator=(DeviceBuffer&& o) noexcept
      {
         std::swap(data_, o.data_);
         std::swap(size_, o.size_);
         return *this;
      }

      // Contents are lost if the buffer grows; never shrinks
      void create(size_t n_bytes)
      {
         if(n_bytes <= size_) return;
         if(data_ != nullptr) CUDA_CHECK(cudaFree(data_));
         data_ = nullptr;
         size_ = 0;
         CUDA_CHECK(cudaMalloc(&data_, n_bytes));
         size_ = n_bytes;
      }

      void* data() noexcept { return data_; }
   };

} // namespace cuda
} // namespace perceive
