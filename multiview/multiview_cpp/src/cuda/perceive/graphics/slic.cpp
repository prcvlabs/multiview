
#include "slic.hpp"
#include "slic_impl.hpp"
#include <cuda.h>
#include <iostream>

namespace perceive::cuda::slic
{
struct SLIC::Pimpl
{
   unsigned step{0};
   float compactness{0.0f};
   ClusterImage clusters;
   ColorImage input;
   std::unique_ptr<LabelImage> labels{make_unique<LabelImage>()};
   std::unique_ptr<LabelImage> new_labels{make_unique<LabelImage>()};
   size_t npixels{0};
   float* staging_lab_pixels{nullptr};
   int* staging_labels{nullptr};
   cudaStream_t stream{0};
   std::mutex lock;

   Pimpl();
   ~Pimpl();
   void
   process_input(const Vec3fImage& image, unsigned step, float compactness);
   void get_output(IntImage& output, unsigned& n_labels);
   void init_clusters();
   void assign_pixels();
   void calc_cluster_centers();
   void enforce_connectivity();
};

SLIC::Pimpl::~Pimpl()
{
   if(staging_lab_pixels) {
      CUDA_CHECK(cudaFreeHost(staging_lab_pixels));
      staging_lab_pixels = nullptr;
   }
   if(staging_labels) {
      CUDA_CHECK(cudaFreeHost(staging_labels));
      staging_labels = nullptr;
   }
}

SLIC::Pimpl::Pimpl() { CUDA_CHECK(cudaStreamCreate(&stream)); }

template<typename T> T* device_ptr(T* host_ptr)
{
   T* result{nullptr};
   CUDA_CHECK(cudaHostGetDevicePointer(&result, host_ptr, 0));
   return result;
}

void SLIC::Pimpl::process_input(const Vec3fImage& image,
                                unsigned step,
                                float compactness)
{
   this->step        = step;
   this->compactness = compactness;

   input.resize(image.width, image.height);
   labels->resize(image.width, image.height);
   new_labels->resize(image.width, image.height);

   size_t new_npixels{image.width * image.height};
   if(new_npixels != npixels) {
      npixels = new_npixels;
      if(staging_lab_pixels) CUDA_CHECK(cudaFree(staging_lab_pixels));
      if(staging_labels) CUDA_CHECK(cudaFree(staging_labels));
      CUDA_CHECK(
          cudaHostAlloc(&staging_lab_pixels, npixels * sizeof(float) * 3, 0));
      CUDA_CHECK(cudaHostAlloc(&staging_labels, npixels * sizeof(int), 0));
   }

   for(unsigned y{0}; y < image.height; ++y) {
      const auto row{image.row_ptr(y)};
      for(unsigned x{0}; x < image.width; ++x) {
         const Vector3f& in_pixel{row[x]};
         float* out_pixel = staging_lab_pixels + (3 * (y * image.width + x));
         for(int i{0}; i < 3; ++i) { out_pixel[i] = in_pixel[i]; }
      }
   }

   import_lab_from_linear_pixels(device_ptr(staging_lab_pixels), input, stream);
}

void SLIC::Pimpl::get_output(IntImage& output, unsigned& n_labels)
{
   export_labels_to_linear_pixels(*labels, device_ptr(staging_labels), stream);
   CUDA_CHECK(cudaStreamSynchronize(stream));

   n_labels = clusters.width() * clusters.height();

   output.resize(labels->width(), labels->height());
   int* in_pixel{staging_labels};
   for(unsigned y{0}; y < labels->height(); ++y) {
      for(unsigned x{0}; x < labels->width(); ++x) {
         output(x, y) = *(in_pixel++);
      }
   }
}

void SLIC::Pimpl::init_clusters()
{
   const GridSampling x_samples{input.width(), step};
   const GridSampling y_samples{input.height(), step};
   clusters.resize(x_samples.n_samples(), y_samples.n_samples());
   slic::init_clusters(clusters, input, x_samples, y_samples, stream);
}

void SLIC::Pimpl::assign_pixels()
{
   const GridSampling x_samples{input.width(), step};
   const GridSampling y_samples{input.height(), step};
   const float xy_weight{compactness * compactness / (step * step)};
   slic::assign_pixels(
       clusters, input, *labels, x_samples, y_samples, xy_weight, stream);
}

void SLIC::Pimpl::calc_cluster_centers()
{
   slic::calc_cluster_centers(clusters, step, input, *labels, stream);
}

void SLIC::Pimpl::enforce_connectivity()
{
   constexpr int n_iterations{2};
   for(int i{0}; i < n_iterations; ++i) {
      slic::enforce_connectivity(clusters, input, *labels, *new_labels, stream);
      labels.swap(new_labels);
   }
}

SLIC::SLIC() { pimpl_ = std::make_unique<Pimpl>(); }
SLIC::~SLIC() = default;

SLIC::Result SLIC::calc_labels(const Vec3fImage& image,
                               unsigned spixel_area,
                               float compactness) const
{
   lock_guard<std::mutex> padlock{pimpl_->lock};

   auto step = unsigned(sqrt(spixel_area) + 0.5);
   pimpl_->process_input(image, step, compactness);

   pimpl_->init_clusters();

   for(unsigned i{0}; i < 9; ++i) {
      pimpl_->assign_pixels();
      pimpl_->calc_cluster_centers();
   }
   pimpl_->assign_pixels();

   pimpl_->enforce_connectivity();

   SLIC::Result result{};
   pimpl_->get_output(result.labels, result.n_labels);
   return result;
}
} // namespace perceive::cuda::slic
