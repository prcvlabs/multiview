
#include "image-layout.hpp"
#include "strided-loop.hpp"

#include <cstdint>
#include <cuda_runtime.h>
#include <stdexcept>

namespace perceive
{
namespace cuda
{
   // cv::remap quantizes map coordinates to 1/32 of a pixel
   // (INTER_BITS = 5); doing the same keeps the results within rounding.
   constexpr int k_inter_bits     = 5;
   constexpr int k_inter_tab_size = 1 << k_inter_bits;

   template<int n_channels> struct Pixel
   {
      uint8_t elements[n_channels];
   };

   struct BorderValue
   {
      float value[4];
   };

   template<int n_channels>
   __device__ static void
   accumulate(const ImageLayout<const Pixel<n_channels>>& src,
              const int x,
              const int y,
              const float weight,
              const BorderValue& border,
              float* sum)
   {
      if(x >= 0 && y >= 0 && x < int(src.width) && y < int(src.height)) {
         const auto& pixel = src.pixel(unsigned(x), unsigned(y));
         for(int c = 0; c < n_channels; ++c)
            sum[c] += weight * pixel.elements[c];
      } else {
         for(int c = 0; c < n_channels; ++c) sum[c] += weight * border.value[c];
      }
   }

   template<int n_channels>
   __global__ void
   kern_remap_bilinear(const ImageLayout<const Pixel<n_channels>> src,
                       const ImageLayout<const float> mapx,
                       const ImageLayout<const float> mapy,
                       ImageLayout<Pixel<n_channels>> dst,
                       const BorderValue border)
   {
      strided_loop_2d(dst.width, dst.height, [&](unsigned x, unsigned y) {
         const int qx  = __float2int_rn(mapx.pixel(x, y) * k_inter_tab_size);
         const int qy  = __float2int_rn(mapy.pixel(x, y) * k_inter_tab_size);
         const int x0  = qx >> k_inter_bits;
         const int y0  = qy >> k_inter_bits;
         const float a = float(qx & (k_inter_tab_size - 1)) / k_inter_tab_size;
         const float b = float(qy & (k_inter_tab_size - 1)) / k_inter_tab_size;

         float sum[n_channels] = {};
         accumulate<n_channels>(src, x0, y0, (1 - a) * (1 - b), border, sum);
         accumulate<n_channels>(src, x0 + 1, y0, a * (1 - b), border, sum);
         accumulate<n_channels>(src, x0, y0 + 1, (1 - a) * b, border, sum);
         accumulate<n_channels>(src, x0 + 1, y0 + 1, a * b, border, sum);

         auto& out = dst.pixel(x, y);
         for(int c = 0; c < n_channels; ++c)
            out.elements[c] = uint8_t(min(max(__float2int_rn(sum[c]), 0), 255));
      });
   }

   template<int n_channels>
   __global__ void
   kern_remap_nearest(const ImageLayout<const Pixel<n_channels>> src,
                      const ImageLayout<const float> mapx,
                      const ImageLayout<const float> mapy,
                      ImageLayout<Pixel<n_channels>> dst,
                      const BorderValue border)
   {
      strided_loop_2d(dst.width, dst.height, [&](unsigned x, unsigned y) {
         float sum[n_channels] = {};
         accumulate<n_channels>(src,
                                __float2int_rn(mapx.pixel(x, y)),
                                __float2int_rn(mapy.pixel(x, y)),
                                1.0f,
                                border,
                                sum);

         auto& out = dst.pixel(x, y);
         for(int c = 0; c < n_channels; ++c) out.elements[c] = uint8_t(sum[c]);
      });
   }

   template<int n_channels>
   static void launch_remap(const ImageLayout<const Pixel<n_channels>>& src,
                            const ImageLayout<const float>& mapx,
                            const ImageLayout<const float>& mapy,
                            const ImageLayout<Pixel<n_channels>>& dst,
                            const bool bilinear,
                            const BorderValue& border,
                            cudaStream_t stream)
   {
      const dim3 block_size{32, 8};
      const dim3 grid_size{(dst.width + block_size.x - 1) / block_size.x,
                           (dst.height + block_size.y - 1) / block_size.y};
      if(bilinear)
         kern_remap_bilinear<n_channels><<<grid_size, block_size, 0, stream>>>(
             src, mapx, mapy, dst, border);
      else
         kern_remap_nearest<n_channels><<<grid_size, block_size, 0, stream>>>(
             src, mapx, mapy, dst, border);
   }

   void cuda_remap(const uint8_t* src,
                   unsigned src_width,
                   unsigned src_height,
                   const float* mapx,
                   const float* mapy,
                   uint8_t* dst,
                   unsigned dst_width,
                   unsigned dst_height,
                   int n_channels,
                   bool bilinear,
                   const uint8_t border_value[4],
                   cudaStream_t stream)
   {
      const ImageLayout<const float> mapx_layout{dst_width, dst_height, mapx};
      const ImageLayout<const float> mapy_layout{dst_width, dst_height, mapy};
      const BorderValue border{{float(border_value[0]),
                                float(border_value[1]),
                                float(border_value[2]),
                                float(border_value[3])}};

      auto launch = [&](auto pixel) {
         using Pix = decltype(pixel);
         launch_remap(ImageLayout<const Pix>{src_width,
                                             src_height,
                                             reinterpret_cast<const Pix*>(src)},
                      mapx_layout,
                      mapy_layout,
                      ImageLayout<Pix>{
                          dst_width, dst_height, reinterpret_cast<Pix*>(dst)},
                      bilinear,
                      border,
                      stream);
      };

      switch(n_channels) {
      case 1: launch(Pixel<1>{}); break;
      case 2: launch(Pixel<2>{}); break;
      case 3: launch(Pixel<3>{}); break;
      case 4: launch(Pixel<4>{}); break;
      default:
         throw std::runtime_error("remap: n-channels must be 1, 2, 3 or 4");
      }
   }

} // namespace cuda
} // namespace perceive
