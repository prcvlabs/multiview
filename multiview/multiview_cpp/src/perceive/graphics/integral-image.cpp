
#include "integral-image.hpp"

namespace perceive
{
template<typename T, typename U>
void make_integral_image_T(const T* data, unsigned w, unsigned h, U* integral)
{
   // First copy across the data
   unsigned sz = w * h;
   for(unsigned i = 0; i < sz; ++i) integral[i] = data[i];

   // sum across the columns
   for(unsigned y = 0; y < h; ++y) {
      unsigned row = y * w;
      for(unsigned x = 1; x < w; ++x)
         integral[row + x] += integral[row + x - 1];
   }

   // sum down the rows
   for(unsigned y = 1; y < h; ++y) {
      unsigned row = y * w;
      for(unsigned x = 0; x < w; ++x)
         integral[row + x] += integral[row - w + x];
   }
}

void make_integral_image(const uint8_t* data, unsigned w, unsigned h,
                         uint32_t* integral)
{
   make_integral_image_T(data, w, h, integral);
}

void make_integral_image(const uint32_t* data, unsigned w, unsigned h,
                         uint32_t* integral)
{
   make_integral_image_T(data, w, h, integral);
}

void make_integral_image(const float* data, unsigned w, unsigned h,
                         float* integral)
{
   make_integral_image_T(data, w, h, integral);
}

} // namespace perceive
