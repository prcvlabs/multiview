
#pragma once

#include <opencv2/opencv.hpp>
#include <type_traits>

namespace perceive::cuda
{
#ifdef WITH_CUDA

class Remapper
{
   // Regions of zero copy memory for input and output data.
   mutable cv::cuda::HostMem input_buffer{cv::cuda::HostMem::AllocType::SHARED};
   mutable cv::cuda::HostMem output_buffer{
       cv::cuda::HostMem::AllocType::SHARED};

   mutable cv::cuda::Stream stream;
   mutable std::mutex lock;

 public:
   cv::Mat remap(const cv::Mat& distorted,
                 const cv::cuda::GpuMat mapx,
                 const cv::cuda::GpuMat mapy,
                 int interpolation_method) const;
};

#else

class Remapper
{};

#endif

} // namespace perceive::cuda
