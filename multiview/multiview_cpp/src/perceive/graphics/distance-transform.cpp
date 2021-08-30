
#include "distance-transform.hpp"

namespace perceive
{
void distance_transform(uint8_t* grey,
                        uint32_t width,
                        uint32_t height,
                        bool l2_norm,
                        uint8_t cap)
{
   const int w  = int(width);
   const int h  = int(height);
   uint8_t* ptr = grey;

   {
      uint8_t* end = ptr + w * h;
      uint64_t cap64
          = std::min(uint64_t(cap), uint64_t(sqrt(w * w + h * h) + 1.0));
      uint8_t cap8 = uint8_t(
          std::min(cap64, uint64_t(std::numeric_limits<uint8_t>::max())));
      for(auto p = ptr; p != end; ++p) *p = (*p >= 127 ? 0 : cap8);
   }

   int row = 0, col = 0; // the row/col we're working on
   uint8_t* dest = grey;
   uint8_t min;
   const double root_2 = sqrt(2.0);

   auto treat = [&]() {
      assert(col < w);
      assert(col >= 0);
      assert(row < h);
      assert(row >= 0);
      assert(dest == ptr + row * w + col);
      min = *dest;
      // l1-norm
      if(col > 0 && (*(dest - 1) + 1.0) < min) min = *(dest - 1) + 1;
      if(col < w - 1 && (*(dest + 1) + 1.0) < min) min = *(dest + 1) + 1;
      if(row > 0 && (*(dest - w) + 1.0) < min) min = *(dest - w) + 1;
      if(row < h - 1 && (*(dest + w) + 1.0) < min) min = *(dest + w) + 1;
      if(l2_norm) {
         if(col > 0 && row > 0 && (*(dest - 1 - w) + root_2) < min)
            min = uint8_t(*(dest - 1 - w) + root_2);
         if(col < w - 1 && row > 0 && (*(dest + 1 - w) + root_2) < min)
            min = uint8_t(*(dest + 1 - w) + root_2);
         if(col > 0 && row < h - 1 && (*(dest - 1 + w) + root_2) < min)
            min = uint8_t(*(dest - 1 + w) + root_2);
         if(col < w - 1 && row < h - 1 && (*(dest + 1 + w) + root_2) < min)
            min = uint8_t(*(dest + 1 + w) + root_2);
      }
      *dest = min;
   };

   // left-to-right-to-left
   auto dt_row = [&]() {
      assert(dest == ptr + row * w);
      for(col = 0; col < w; ++col) {
         treat();
         dest++;
      }
      assert(dest == ptr + row * w + w);
      for(col = w - 1; col >= 0; --col) {
         --dest;
         treat();
      }
      assert(dest == ptr + row * w);
   };

   // top-to-bottom-to-top
   dest = ptr;
   for(row = 0; row < h; ++row) {
      dt_row();
      dest += w;
   }
   for(row = h - 1; row >= 0; --row) {
      dest -= w;
      dt_row();
   }
}

} // namespace perceive
