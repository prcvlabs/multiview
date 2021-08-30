
#pragma once

#include "perceive/graphics/image-container.hpp"

namespace perceive::cuda::slic
{
#ifdef WITH_CUDA

struct SLIC
{
   struct Result
   {
      IntImage labels;
      unsigned n_labels;
   };

 private:
   struct Pimpl;
   mutable std::unique_ptr<Pimpl> pimpl_;

 public:
   SLIC();
   ~SLIC();
   Result calc_labels(const Vec3fImage& image,
                      unsigned spixel_area,
                      float compactness) const;
};

#else

struct SLIC
{
   struct Result
   {
      IntImage labels;
      unsigned n_labels;
   };

 public:
   SLIC() {}
   ~SLIC() {}
   Result calc_labels(const Vec3fImage& image,
                      unsigned spixel_area,
                      float compactness) const
   {
      throw std::runtime_error("CUDA is required");
   }
};

#endif // WITH_CUDA

} // namespace perceive::cuda::slic
