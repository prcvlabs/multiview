
#include "remap.hpp"

#include <opencv2/cudawarping.hpp>

namespace perceive::cuda
{
cv::Mat Remapper::remap(const cv::Mat& distorted,
                        const cv::cuda::GpuMat mapx,
                        const cv::cuda::GpuMat mapy,
                        int interpolation_method) const
{
   lock_guard<std::mutex> padlock(lock);

   input_buffer.create(distorted.size(), distorted.type());
   output_buffer.create(mapx.size(), distorted.type());
   cv::Mat in_host{input_buffer.createMatHeader()};
   cv::Mat out_host{output_buffer.createMatHeader()};
   cv::cuda::GpuMat in_gpu{input_buffer.createGpuMatHeader()};
   cv::cuda::GpuMat out_gpu{output_buffer.createGpuMatHeader()};

   cv::cuda::remap(in_gpu,
                   out_gpu,
                   mapx,
                   mapy,
                   interpolation_method,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255),
                   stream);

   stream.waitForCompletion();

   return cv::Mat{out_gpu};
}

} // namespace perceive::cuda
