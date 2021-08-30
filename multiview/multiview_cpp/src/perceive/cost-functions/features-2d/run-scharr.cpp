
#include "features-2d.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/colour-set.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace perceive
{
// ---------------------------------------------------------------------- Scharr

void run_scharr(ImageFeatures2d& ss,
                const cv::Mat& grey,
                const ImageFeatures2d::Params& p,
                std::function<bool(void)> is_canceled)
{
   Expects(ss.w != 0);
   Expects(ss.h != 0);

   const auto w = ss.w;
   const auto h = ss.h;

   if(is_canceled()) return;

   unsigned blur_sz  = p.scharr_blur_sz;
   double blur_sigma = p.scharr_blur_sigma;

   // Apply Gaussian blur
   auto sz = cv::Size(int(blur_sz), int(blur_sz));
   cv::GaussianBlur(grey, grey, sz, blur_sigma, blur_sigma, cv::BORDER_DEFAULT);

   /// Generate grad_x and grad_y
   cv::Mat grad_x, grad_y;
   cv::Mat abs_grad_x, abs_grad_y;

   /// Gradient X and Y
   auto ddepth = CV_32F; // produce floating-point images
   Scharr(grey, grad_x, ddepth, 1, 0);
   Scharr(grey, grad_y, ddepth, 0, 1);

   if(is_canceled()) return;

   // Convert to a field...
   auto& field  = ss.scharr;
   auto& invmag = ss.scharr_invmag;
   field.resize(grey.cols, grey.rows);
   invmag.resize(grey.cols, grey.rows);

   // Pack result into our field...
   auto dst = field.data();
   auto d2  = invmag.data();
   std::vector<real> mags;
   mags.reserve(size_t(grey.rows * grey.cols));
   for(int y = 0; y < grey.rows; ++y) {
      const float* src_x = grad_x.ptr<const float>(y);
      const float* src_y = grad_y.ptr<const float>(y);
      for(int x = 0; x < grey.cols; ++x) {
         dst->x       = real(*src_x++);
         dst->y       = real(*src_y++);
         auto norm    = dst->norm();
         auto mag_inv = 1.0 / norm;
         mags.emplace_back(norm);
         // if(!std::isfinite(mag_inv)) {
         //     cout << format("mag_inv = {}, dst = {}",
         //                    mag_inv, dst->to_string())
         //          << endl;
         //     FATAL("kBAM!");
         // }
         *d2++ = float(mag_inv);
         dst++;
      }
   }

   // Calculate statistics
   ss.scharr_mag_stats = calc_sample_statistics(mags.begin(), mags.end());

   // Make the threshold image
   const auto threshold
       = ss.scharr_mag_stats.median
         + p.scharr_threshold_absdevs * ss.scharr_mag_stats.absdev;

   {
      auto& im = ss.scharr_binary;
      im.resize(field.width, field.height);
      im.resize(w, h);
      im.zero();
      for(auto y = 0u; y < field.height; ++y) {
         for(auto x = 0u; x < field.width; ++x) {
            Vector2 v = field(x, y);
            if(v.norm() > threshold) {
               auto theta = atan2(v.y, v.x);
               if(theta < 0) theta += M_PI;
               im(x, y) = angle_to_colour(theta);
            }
         }
      }
   }
}

} // namespace perceive
