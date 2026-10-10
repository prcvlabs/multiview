
#include "cuda/perceive/graphics/remap.hpp"
#include "perceive/utils/cuda-spec.hpp"
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include <opencv2/imgproc.hpp>

namespace perceive
{
// ----------------------------------------------------------------

#ifdef WITH_CUDA
CATCH_TEST_CASE("CudaRemap", "[cuda_remap]")
{
   CATCH_SECTION("remap")
   {
      if(!cuda::cuda_is_available()) {
         WARN("no CUDA device found, skipping CudaRemap");
         return;
      }

      const int w = 640, h = 480;
      const cv::Scalar border_value(255, 255, 255);

      cv::Mat image(h, w, CV_8UC3);
      cv::RNG rng(0x5eed);
      rng.fill(image, cv::RNG::UNIFORM, 0, 256);

      // Barrel distortion plus a rotation, scaled so that the edges map
      // outside the source image (exercising the constant border).
      cv::Mat mapx(h, w, CV_32FC1);
      cv::Mat mapy(h, w, CV_32FC1);
      const double cx    = 0.5 * w;
      const double cy    = 0.5 * h;
      const double theta = 0.1;
      const double k1    = 0.25;
      for(int y = 0; y < h; ++y) {
         for(int x = 0; x < w; ++x) {
            const double dx = (x - cx) / cx, dy = (y - cy) / cy;
            const double s = 1.1 * (1.0 + k1 * (dx * dx + dy * dy));
            const double u = s * (std::cos(theta) * dx - std::sin(theta) * dy);
            const double v = s * (std::sin(theta) * dx + std::cos(theta) * dy);
            mapx.at<float>(y, x) = float(cx + u * cx);
            mapy.at<float>(y, x) = float(cy + v * cy);
         }
      }

      cuda::RemapMaps maps;
      maps.upload(mapx, mapy);
      cuda::Remapper remapper;

      for(const int interpolation : {cv::INTER_LINEAR, cv::INTER_NEAREST}) {
         cv::Mat cpu_output;
         cv::remap(image,
                   cpu_output,
                   mapx,
                   mapy,
                   interpolation,
                   cv::BORDER_CONSTANT,
                   border_value);
         const cv::Mat gpu_output
             = remapper.remap(image, maps, interpolation, border_value);

         CATCH_REQUIRE(gpu_output.size() == cpu_output.size());
         CATCH_REQUIRE(gpu_output.type() == cpu_output.type());

         cv::Mat diff;
         cv::absdiff(gpu_output, cpu_output, diff);
         double max_diff = 0.0;
         cv::minMaxLoc(diff.reshape(1), nullptr, &max_diff);
         const double mean_diff = cv::mean(diff.reshape(1))[0];

         INFO(format(
             "remap {}: max abs diff = {}, mean abs diff = {:.4f}",
             (interpolation == cv::INTER_LINEAR ? "bilinear" : "nearest"),
             max_diff,
             mean_diff));
         CATCH_REQUIRE(max_diff <= 1.0);
      }
   }
}
#endif

} // namespace perceive
