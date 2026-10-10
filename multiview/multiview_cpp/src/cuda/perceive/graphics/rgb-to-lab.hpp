
#pragma once

#include "perceive/graphics/image-container.hpp"
#include <opencv2/opencv.hpp>
#include <type_traits>

namespace perceive::cuda
{
#ifdef WITH_CUDA

class ARGBToLABConversion
{
 private:
   struct Pimpl;
   mutable std::unique_ptr<Pimpl> pimpl_;

 public:
   ARGBToLABConversion();
   ARGBToLABConversion(const ARGBToLABConversion&)            = delete;
   ARGBToLABConversion& operator=(const ARGBToLABConversion&) = delete;
   ~ARGBToLABConversion();

   Vec3fImage convert(const ARGBImage& argb) const;
};

#else

class ARGBToLABConversion
{
 public:
   Vec3fImage convert(const ARGBImage& argb) const
   {
      throw std::runtime_error("CUDA is required");
   }
};

#endif

} // namespace perceive::cuda
