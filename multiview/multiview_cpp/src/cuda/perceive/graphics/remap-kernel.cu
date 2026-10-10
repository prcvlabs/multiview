
#include "strided-loop.cuh"

#include <cstdint>
#include <cuda_runtime.h>

namespace perceive
{
namespace cuda
{
   // cv::remap quantizes map coordinates to 1/32 of a pixel
   // (INTER_BITS = 5); doing the same keeps the results within rounding.
   constexpr int k_inter_bits     = 5;
   constexpr int k_inter_tab_size = 1 << k_inter_bits;

   template<typename T> struct RemapImage
   {
      T* data{nullptr};
      unsigned width{0};
      unsigned height{0};
      unsigned row_bytes{0};
   };

   struct BorderValue
   {
      float value[4];
   };

   template<int n_channels>
   __device__ static void accumulate(const RemapImage<const uint8_t>& src,
                                     const int x,
                                     const int y,
                                     const float weight,
                                     const BorderValue& border,
                                     float* sum)
   {
      if(x >= 0 && y >= 0 && x < int(src.width) && y < int(src.height)) {
         const uint8_t* pixel = src.data + y * src.row_bytes + x * n_channels;
         for(int c = 0; c < n_channels; ++c) sum[c] += weight * pixel[c];
      } else {
         for(int c = 0; c < n_channels; ++c) sum[c] += weight * border.value[c];
      }
   }

   template<int n_channels>
   __global__ void kern_remap_bilinear(const RemapImage<const uint8_t> src,
                                       const float* mapx,
                                       const float* mapy,
                                       RemapImage<uint8_t> dst,
                                       const BorderValue border)
   {
      strided_loop_2d(dst.width, dst.height, [&](unsigned x, unsigned y) {
         const unsigned idx = y * dst.width + x;
         const int qx       = __float2int_rn(mapx[idx] * k_inter_tab_size);
         const int qy       = __float2int_rn(mapy[idx] * k_inter_tab_size);
         const int x0       = qx >> k_inter_bits;
         const int y0       = qy >> k_inter_bits;
         const float a = float(qx & (k_inter_tab_size - 1)) / k_inter_tab_size;
         const float b = float(qy & (k_inter_tab_size - 1)) / k_inter_tab_size;

         float sum[n_channels] = {};
         accumulate<n_channels>(src, x0, y0, (1 - a) * (1 - b), border, sum);
         accumulate<n_channels>(src, x0 + 1, y0, a * (1 - b), border, sum);
         accumulate<n_channels>(src, x0, y0 + 1, (1 - a) * b, border, sum);
         accumulate<n_channels>(src, x0 + 1, y0 + 1, a * b, border, sum);

         uint8_t* out = dst.data + y * dst.row_bytes + x * n_channels;
         for(int c = 0; c < n_channels; ++c)
            out[c] = uint8_t(min(max(__float2int_rn(sum[c]), 0), 255));
      });
   }

   template<int n_channels>
   __global__ void kern_remap_nearest(const RemapImage<const uint8_t> src,
                                      const float* mapx,
                                      const float* mapy,
                                      RemapImage<uint8_t> dst,
                                      const BorderValue border)
   {
      strided_loop_2d(dst.width, dst.height, [&](unsigned x, unsigned y) {
         const unsigned idx    = y * dst.width + x;
         float sum[n_channels] = {};
         accumulate<n_channels>(src,
                                __float2int_rn(mapx[idx]),
                                __float2int_rn(mapy[idx]),
                                1.0f,
                                border,
                                sum);

         uint8_t* out = dst.data + y * dst.row_bytes + x * n_channels;
         for(int c = 0; c < n_channels; ++c) out[c] = uint8_t(sum[c]);
      });
   }

   template<int n_channels>
   static void launch_remap(const RemapImage<const uint8_t>& src,
                            const float* mapx,
                            const float* mapy,
                            const RemapImage<uint8_t>& dst,
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
                   unsigned src_row_bytes,
                   const float* mapx,
                   const float* mapy,
                   uint8_t* dst,
                   unsigned dst_width,
                   unsigned dst_height,
                   unsigned dst_row_bytes,
                   int n_channels,
                   bool bilinear,
                   const float border_value[4],
                   cudaStream_t stream)
   {
      const RemapImage<const uint8_t> src_im{
          src, src_width, src_height, src_row_bytes};
      const RemapImage<uint8_t> dst_im{
          dst, dst_width, dst_height, dst_row_bytes};
      const BorderValue border{
          {border_value[0], border_value[1], border_value[2], border_value[3]}};

      switch(n_channels) {
      case 1:
         launch_remap<1>(src_im, mapx, mapy, dst_im, bilinear, border, stream);
         break;
      case 2:
         launch_remap<2>(src_im, mapx, mapy, dst_im, bilinear, border, stream);
         break;
      case 3:
         launch_remap<3>(src_im, mapx, mapy, dst_im, bilinear, border, stream);
         break;
      case 4:
         launch_remap<4>(src_im, mapx, mapy, dst_im, bilinear, border, stream);
         break;
      }
   }

} // namespace cuda
} // namespace perceive
