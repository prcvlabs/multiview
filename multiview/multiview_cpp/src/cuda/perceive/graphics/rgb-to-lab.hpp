
#pragma once

#include "perceive/graphics/image-container.hpp"
#include <opencv2/opencv.hpp>
#include <type_traits>

namespace perceive::cuda
{
#ifdef WITH_CUDA

class ARGBToLABConversion
{
   // Regions of zero copy memory for input and output data.
   mutable cv::cuda::HostMem input_buffer{cv::cuda::HostMem::AllocType::SHARED};
   mutable cv::cuda::HostMem output_buffer{
       cv::cuda::HostMem::AllocType::SHARED};

   mutable cv::cuda::Stream stream;
   mutable std::mutex lock;

 public:
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
