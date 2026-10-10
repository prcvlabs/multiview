
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/disparity/strategies/sgm_disp.hpp"
#include "perceive/utils/cuda-spec.hpp"

#include <opencv2/core.hpp>

namespace perceive
{
#if defined WITH_CUDA && defined USE_SGM

// A synthetic rectified pair: random texture with a background at disparity
// `d_bg` and a square "closer to the camera" at disparity `d_fg`.
CATCH_TEST_CASE("SGMDisparity", "[sgm_disparity]")
{
   CATCH_SECTION("libsgm-synthetic-pair")
   {
      if(!cuda::cuda_is_available()) {
         WARN("no CUDA device found, skipping SGMDisparity");
         return;
      }

      // 240 is not a multiple of 32, which exercises the padding path
      const int w    = 320;
      const int h    = 240;
      const int d_bg = 8;
      const int d_fg = 24;
      const cv::Rect fg_right(120, 70, 100, 100); // square in the right image

      cv::Mat left(h, w, CV_8UC1);
      cv::RNG rng(0x5eed);
      rng.fill(left, cv::RNG::UNIFORM, 1, 256); // 0 is masked out by libSGM

      cv::Mat right(h, w, CV_8UC1);
      for(int y = 0; y < h; ++y) {
         for(int x = 0; x < w; ++x) {
            const int d = fg_right.contains(cv::Point(x, y)) ? d_fg : d_bg;
            right.at<uint8_t>(y, x)
                = left.at<uint8_t>(y, std::min(x + d, w - 1));
         }
      }

      // Score pixels well inside each region (in left-image coordinates)
      const cv::Rect fg_left(
          fg_right.x + d_fg, fg_right.y, fg_right.width, fg_right.height);
      const cv::Rect fg_inner(fg_left.x + 16,
                              fg_left.y + 16,
                              fg_left.width - 32,
                              fg_left.height - 32);
      const cv::Rect bg_inner(32, 16, 64, h - 32); // left strip

      auto score = [&](const cv::Mat& disp, const cv::Rect& roi, int expect) {
         int good = 0;
         for(int y = roi.y; y < roi.br().y; ++y)
            for(int x = roi.x; x < roi.br().x; ++x)
               if(std::fabs(disp.at<float>(y, x) - float(expect)) <= 1.0f)
                  ++good;
         return double(good) / double(roi.area());
      };

      for(const bool use_16bit : {false, true}) {
         disparity::SGMDisparity::Params p;
         p.disparity_size      = 64;
         p.use_16bit_greyscale = use_16bit;

         cv::Mat l = left;
         cv::Mat r = right;
         if(use_16bit) {
            left.convertTo(l, CV_16UC1, 256.0);
            right.convertTo(r, CV_16UC1, 256.0);
         }

         cv::Mat disp;
         disparity::SGMDisparity::calculate(p, l, r, disp);
         CATCH_REQUIRE(disp.type() == CV_32FC1);
         CATCH_REQUIRE(disp.rows == h);
         CATCH_REQUIRE(disp.cols == w);

         const double fg = score(disp, fg_inner, d_fg);
         const double bg = score(disp, bg_inner, d_bg);
         INFO(format("libSGM {}-bit {}x{}: {:.1f}% of foreground pixels at "
                     "d={}, {:.1f}% of background pixels at d={}",
                     (use_16bit ? 16 : 8),
                     w,
                     h,
                     100.0 * fg,
                     d_fg,
                     100.0 * bg,
                     d_bg));
         CATCH_REQUIRE(fg > 0.95);
         CATCH_REQUIRE(bg > 0.95);
      }
   }
}

#endif
} // namespace perceive
