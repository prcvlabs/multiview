
#pragma once

#include <cstring>
#include <cuda_runtime.h>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>

#define CUDA_CHECK(expr) \
   perceive::cuda::slic::cuda_check_impl(expr, #expr, __FILE__, __LINE__)

namespace perceive
{
namespace cuda
{
   namespace slic
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

      struct Color
      {
         float elements[3];
         Color() = default;
         __device__ Color(float a, float b, float c)
             : elements{a, b, c}
         {}
         __device__ float& operator[](size_t n) { return elements[n]; }
         __device__ const float& operator[](size_t n) const
         {
            return elements[n];
         }
      };

      struct Cluster
      {
         Color color;
         float x;
         float y;
      };

      template<typename T> struct ImageLayout
      {
       public:
         unsigned width{0};
         unsigned height{0};
         T* pixels{nullptr};

         constexpr static size_t alloc_objects(unsigned width, unsigned height)
         {
            return size_t{width} * size_t{height};
         }
         constexpr static size_t alloc_bytes(unsigned width, unsigned height)
         {
            return sizeof(T) * alloc_objects(width, height);
         }
         __device__ __host__ unsigned pix_index(unsigned x, unsigned y) const
         {
            return width * y + x;
         }
         __device__ __host__ const T& pixel(const unsigned x,
                                            const unsigned y) const
         {
            return pixels[pix_index(x, y)];
         }
         __device__ __host__ T& pixel(const unsigned x, const unsigned y)
         {
            return pixels[pix_index(x, y)];
         }

         ImageLayout(const unsigned w, const unsigned h, T* pixels)
             : width(w)
             , height(h)
             , pixels(pixels)
         {}
         ImageLayout() = default;
      };

      template<typename T> struct DeviceImage
      {
         static_assert(std::is_trivial<T>::value, "Pixel must be trivial type");

       private:
         ImageLayout<T> layout_;

       public:
         DeviceImage() {}
         ~DeviceImage()
         {
            if(layout_.pixels) CUDA_CHECK(cudaFree(layout_.pixels));
         }
         DeviceImage(unsigned w, unsigned h) { resize(w, h); }
         DeviceImage(const DeviceImage& other) { *this = other; }
         DeviceImage& operator=(const DeviceImage& other)
         {
            resize(other.width(), other.height());
            CUDA_CHECK(cudaMemcpy(
                layout_.pixels,
                other.layout_.pixels,
                ImageLayout<T>::alloc_bytes(other.width(), other.height())));
            return *this;
         }
         void copy_to_host(const DeviceImage& src) {}
         unsigned width() const { return layout_.width; }
         unsigned height() const { return layout_.height; }
         void resize(const unsigned w, const unsigned h)
         {
            if(w != layout_.width || h != layout_.height) {
               if(layout_.pixels) {
                  CUDA_CHECK(cudaFree(layout_.pixels));
                  layout_.pixels = nullptr;
               }
               T* pixels;
               CUDA_CHECK(
                   cudaMalloc(&pixels, ImageLayout<T>::alloc_bytes(w, h)));
               layout_ = ImageLayout<T>(w, h, pixels);
            }
         }
         const T& pixel(const unsigned x, const unsigned y) const
         {
            return layout_.pixel(x, y);
         }
         T& pixel(const unsigned x, const unsigned y)
         {
            return layout_.pixel(x, y);
         }
         const ImageLayout<T>& layout() const { return layout_; }
      };

      using ColorImage   = DeviceImage<Color>;
      using LabelImage   = DeviceImage<int>;
      using ClusterImage = DeviceImage<Cluster>;

      struct GridSampling
      {
         GridSampling(unsigned n_pixels, unsigned step_size)
             : n_pixels{n_pixels}
             , step_size{step_size}
         {}
         __device__ __host__ unsigned n_samples() const
         {
            return unsigned(double(n_pixels) / double(step_size) - 0.5);
         }

         __device__ __host__ unsigned sample_center(unsigned i) const
         {
            double frac = (i + 0.5) / n_samples();
            return unsigned(frac * n_pixels);
         }
         __device__ __host__ unsigned sample_index(unsigned position) const
         {
            return unsigned(position * n_samples() / n_pixels);
         }

       private:
         unsigned n_pixels{0};
         unsigned step_size{0};
      };

      // kernel runner functions

      void init_clusters(ClusterImage& clusters,
                         const ColorImage& input_image,
                         const GridSampling& x_samples,
                         const GridSampling& y_samples,
                         cudaStream_t stream = 0);

      void assign_pixels(const ClusterImage& clusters,
                         const ColorImage& input_image,
                         LabelImage& labels,
                         const GridSampling& x_samples,
                         const GridSampling& y_samples,
                         const float xy_weight,
                         cudaStream_t stream = 0);

      void calc_cluster_centers(ClusterImage& clusters,
                                const unsigned step,
                                const ColorImage& colors,
                                const LabelImage& labels,
                                cudaStream_t stream = 0);

      void enforce_connectivity(const ClusterImage& clusters,
                                const ColorImage& colors,
                                const LabelImage& labels,
                                LabelImage& new_labels,
                                cudaStream_t stream = 0);

      void import_lab_from_linear_pixels(const float* lab_pixels,
                                         ColorImage& dest,
                                         cudaStream_t stream = 0);

      void export_labels_to_linear_pixels(const LabelImage& src,
                                          int* label_pixels,
                                          cudaStream_t stream = 0);
   } // namespace slic
} // namespace cuda
} // namespace perceive
