
#include "stdinc.hpp"

#include "slic-data.hpp"

#include "perceive/contrib/SLIC/SLIC.h"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"

#define This SlicData

namespace perceive::calibration
{
static void draw_magnified_pixel(ARGBImage& contours,
                                 const int x,
                                 const int y,
                                 const unsigned mag, // magnification factor
                                 const uint32_t k) noexcept
{
   Point2 p{x * int(mag), y * int(mag)};
   for(auto dy = 0u; dy < mag; ++dy) {
      for(auto dx = 0u; dx < mag; ++dx) {
         const int mx = p.x + int(dx);
         const int my = p.y + int(dy);
         if(contours.in_bounds(mx, my)) contours(mx, my) = k;
      }
   }
}

// ------------------------------------------------------------------------ init

bool This::init(const ARGBImage image,
                unsigned superpixel_size,
                real compactness,
                std::function<bool()> is_canceled) noexcept
{
   if(is_canceled()) return false;

   // Prepare
   input_image           = image;
   this->superpixel_size = superpixel_size;
   this->compactness     = compactness;
   slic_info.clear();

   if(is_canceled()) return false;

   const ARGBImage& argb = image;
   const unsigned w = argb.width, h = argb.height;
   this->contours = argb;
   this->labels.resize(w, h, w);

   if(is_canceled()) return false;

   { // Superpixels!
      SLIC slic;
      const unsigned int* ubuff = argb.pixels;
      int32_t* klabels          = labels.data();
      int num_k                 = 0;
      slic.DoSuperpixelSegmentation_ForGivenSuperpixelSize(ubuff,
                                                           int(w),
                                                           int(h),
                                                           klabels,
                                                           num_k,
                                                           int(superpixel_size),
                                                           compactness);
      this->n_labels = unsigned(num_k);
   }

   if(is_canceled()) return false;

   { // slic-info
      slic_info.clear();
      slic_info.resize(n_labels);

      for(auto y = 0; y < int(h); ++y) {
         if(is_canceled()) return false;

         for(auto x = 0; x < int(w); ++x) {
            const size_t l0 = size_t(labels(x, y));
            slic_info[l0].inliers.emplace_back(x, y);
            slic_info[l0].center += Vector2(x, y);

            // Is this a border spixel??
            bool is_border = false;
            for(const auto& dxy : eight_connected) {
               const auto p = Point2(x + dxy.first, y + dxy.second);
               if(!labels.in_bounds(p)) continue;
               if(labels(p) != int(l0)) {
                  slic_info[l0].borders.emplace_back(x, y);
                  break;
               }
            }
         }
      }

      // Turn center 'sums' into averages. (i.e., find the actual center)
      for(auto i = 0u; i < n_labels; ++i)
         slic_info[i].center /= real(slic_info[i].inliers.size());
   }

   if(is_canceled()) return false;

   { // Draw the contours
      const unsigned ow = w;
      const unsigned oh = h;
      const unsigned f  = 4; // 4x the size

      contours.resize(ow * f, oh * f, ow * f);

      // Copy in the data
      for(auto y = 0u; y < contours.height; ++y) {
         if(is_canceled()) return false;
         for(auto x = 0u; x < contours.width; ++x)
            contours(x, y) = argb(x / f, y / f);
      }

      { // Draw the labels
         for(auto i = 0u; i < n_labels; ++i) {
            const auto s_info = slic_info[i];
            const auto center = s_info.center * double(f);
            render_string(
                contours, format("{}", i), to_pt2(center), k_yellow, k_black);
         }
      }

      { // Draw the borders
         for(const auto& info : slic_info) {
            if(is_canceled()) return false;
            for(const auto& p : info.borders)
               draw_magnified_pixel(contours, p.x, p.y, f, k_white);
         }
      }
   }

   return true;
}

} // namespace perceive::calibration
