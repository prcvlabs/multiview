
#include <cmath>
#include <cuda_runtime.h>
#include <iostream>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

namespace perceive
{
namespace cuda
{
   struct Color
   {
      float elements[3];
      __device__ Color(float a, float b, float c)
          : elements{a, b, c}
      {}
      __device__ float& operator[](size_t n) { return elements[n]; }
      __device__ const float& operator[](size_t n) const { return elements[n]; }
   };

   __device__ static inline Color rgb_to_xyz(const Color& in) noexcept
   {
      const auto R = in[0], G = in[1], B = in[2];

      const auto r = (R <= 0.04045) ? R / 12.92 : pow((R + 0.055) / 1.055, 2.4);
      const auto g = (G <= 0.04045) ? G / 12.92 : pow((G + 0.055) / 1.055, 2.4);
      const auto b = (B <= 0.04045) ? B / 12.92 : pow((B + 0.055) / 1.055, 2.4);

      return Color(r * 0.4124564 + g * 0.3575761 + b * 0.1804375,
                   r * 0.2126729 + g * 0.7151522 + b * 0.0721750,
                   r * 0.0193339 + g * 0.1191920 + b * 0.9503041);
   }

   __device__ static inline Color xyz_to_lab(Color XYZ) noexcept
   {
      const auto X = XYZ[0], Y = XYZ[1], Z = XYZ[2];

      //------------------------
      // XYZ to LAB conversion
      //------------------------
      constexpr double epsilon = 0.008856; // actual CIE standard
      constexpr double kappa   = 7.787;    // actual CIE standard

      constexpr double Xr = 0.950456; // reference white
      constexpr double Yr = 1.0;      // reference white
      constexpr double Zr = 1.088754; // reference white

      const double xr = X / Xr;
      const double yr = Y / Yr;
      const double zr = Z / Zr;

      const auto fx
          = (xr > epsilon) ? std::cbrt(xr) : (kappa * xr + 16.0) / 116.0;
      const auto fy
          = (yr > epsilon) ? std::cbrt(yr) : (kappa * yr + 16.0) / 116.0;
      const auto fz
          = (zr > epsilon) ? std::cbrt(zr) : (kappa * zr + 16.0) / 116.0;

      return Color(116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz));
   }

   __device__ static Color rgb_to_lab(const Color& in) noexcept
   {
      return xyz_to_lab(rgb_to_xyz(in));
   }

   __global__ void image_rgb_to_lab(const uint32_t* in_pixel_start,
                                    const uint32_t* in_pixel_end,
                                    float* out_pixel_start,
                                    size_t out_pixel_size)
   {
      int stride = blockDim.x * gridDim.x;
      int offset = blockIdx.x * blockDim.x + threadIdx.x;

      auto* lab_pixel_bytes = reinterpret_cast<unsigned char*>(out_pixel_start)
                              + (out_pixel_size * offset);
      for(auto* rgb_pixel = in_pixel_start + offset; rgb_pixel < in_pixel_end;
          rgb_pixel += stride) {
         Color rgb_color{((*rgb_pixel >> 16) & 0xff) / 255.0f,
                         ((*rgb_pixel >> 8) & 0xff) / 255.0f,
                         ((*rgb_pixel >> 0) & 0xff) / 255.0f};
         Color lab{rgb_to_lab(rgb_color)};

         auto lab_pixel = reinterpret_cast<float*>(lab_pixel_bytes);
         for(int i = 0; i < 3; ++i) { lab_pixel[i] = lab[i]; }

         lab_pixel_bytes += out_pixel_size * stride;
      }
   }

   void cuda_convert_rgb_to_lab(
       const uint32_t* rgb_pixel_start,
       const uint32_t* rgb_pixel_end,
       float* lab_pixel_start, // pointer to 'L' value of first pixel. L A and
                               // B values are assumed to be contiguous
       size_t lab_pixel_size,  // bytes from one pixel to next
       cv::cuda::Stream& stream)
   {
      cudaStream_t cuda_stream{cv::cuda::StreamAccessor::getStream(stream)};
      dim3 blockSize{256};
      dim3 gridSize{256};

      auto device_ptr = [](auto* host_ptr) {
         decltype(host_ptr) dev_ptr;
         auto err
             = cudaHostGetDevicePointer((void**) &dev_ptr, (void*) host_ptr, 0);
         if(err != cudaSuccess) {
            std::stringstream buf;
            buf << "Cuda error converting host->device pointer "
                << ": " << cudaGetErrorString(err);
            throw std::runtime_error(buf.str());
         }
         return dev_ptr;
      };

      const uint32_t* device_rgb_start{device_ptr(rgb_pixel_start)};
      const uint32_t* device_rgb_end{device_rgb_start
                                     + (rgb_pixel_end - rgb_pixel_start)};
      float* device_lab_start{device_ptr(lab_pixel_start)};

      image_rgb_to_lab<<<blockSize, gridSize, 0, cuda_stream>>>(
          device_rgb_start, device_rgb_end, device_lab_start, lab_pixel_size);

      cudaError_t err = cudaPeekAtLastError();
      if(err != cudaSuccess) {
         std::stringstream buf;
         buf << "Cuda error at " << __FILE__ << ":" << __LINE__ << ": "
             << cudaGetErrorString(err);
         throw std::runtime_error(buf.str());
      }
   }

} // namespace cuda
} // namespace perceive
