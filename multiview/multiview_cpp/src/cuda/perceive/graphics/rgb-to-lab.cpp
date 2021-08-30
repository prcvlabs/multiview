
#include "rgb-to-lab.hpp"

namespace perceive::cuda
{
void cuda_convert_rgb_to_lab(
    const uint32_t* rgb_pixel_start,
    const uint32_t* rgb_pixel_end,
    float* lab_pixel_start, // pointer to 'L' value of first pixel. L A and
                            // B values are assumed to be contiguous
    size_t lab_pixel_size,  // bytes from one pixel to next
    cv::cuda::Stream& stream);

Vec3fImage ARGBToLABConversion::convert(const ARGBImage& argb) const
{
   Expects(argb.width > 0 and argb.height > 0);
   Expects(argb.width == argb.row_stride);

   lock_guard<std::mutex> padlock(lock);

   input_buffer.create(1, int(argb.n_bytes()), CV_8U);
   output_buffer.create(
       1, int(sizeof(Vector3f) * argb.width * argb.height), CV_8U);

   ARGBImage argb_shared{false};
   argb_shared.set_pix(argb.width,
                       argb.height,
                       argb.width,
                       reinterpret_cast<uint32_t*>(input_buffer.data));
   Vec3fImage lab_shared{false};
   lab_shared.set_pix(argb.width,
                      argb.height,
                      argb.width,
                      reinterpret_cast<Vector3f*>(output_buffer.data));

   for(unsigned y = 0; y < argb.height; ++y) {
      memcpy(argb_shared.ptr(y), argb.ptr(y), argb.row_bytes());
   }

   float* lab_start = lab_shared(0, 0).ptr();
   cuda_convert_rgb_to_lab(argb_shared.data(),
                           argb_shared.data() + argb.height * argb.row_stride,
                           lab_start,
                           sizeof(Vector3f),
                           stream);

   stream.waitForCompletion();

   Vec3fImage result{lab_shared};
   return result;
}

} // namespace perceive::cuda
