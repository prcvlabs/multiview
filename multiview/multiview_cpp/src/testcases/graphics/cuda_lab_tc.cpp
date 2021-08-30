
#include <algorithm>
#include <iterator>

#include "cuda/perceive/graphics/rgb-to-lab.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

namespace perceive
{
// ----------------------------------------------------------------

#ifdef WITH_CUDA
CATCH_TEST_CASE("CudaARGBToLAB", "[cuda_argb_to_lab]")
{
   CATCH_SECTION("rgb2lab")
   {
      // Create a giant input image containing every RGB value
      ARGBImage input{256, 256 * 256};
      for(unsigned r{0}; r < 256; ++r) {
         for(unsigned g{0}; g < 256; ++g) {
            for(unsigned b{0}; b < 256; ++b) {
               unsigned x{r};
               unsigned y{256 * g + b};
               input(x, y) = make_colour(r, g, b);
            }
         }
      }

      double epsilon = 1.0;
      cuda::ARGBToLABConversion conversion;

      Vec3fImage cuda_output = conversion.convert(input);
      Vec3fImage cpu_output  = argb_to_LAB_vec3f_im(input);

      CATCH_REQUIRE(cuda_output.width == cpu_output.width);
      CATCH_REQUIRE(cuda_output.height == cpu_output.height);

      double max_distance{0};
      for(unsigned y{0}; y < cuda_output.height; ++y) {
         for(unsigned x{0}; x < cuda_output.width; ++x) {
            auto cuda_pix{cuda_output(x, y)};
            auto cpu_pix{cpu_output(x, y)};
            double sum = 0.0;
            for(int i = 0; i < 3; ++i) {
               double diff = double(std::abs(cuda_pix[i] - cpu_pix[i]));
               sum += diff * diff;
            }
            double distance = std::sqrt(sum);
            max_distance    = std::max(distance, max_distance);
         }
      }
      CATCH_REQUIRE(max_distance < epsilon);
   }
}
#endif
} // namespace perceive
