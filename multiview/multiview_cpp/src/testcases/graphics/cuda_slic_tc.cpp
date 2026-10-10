
#include "cuda/perceive/graphics/rgb-to-lab.hpp"
#include "cuda/perceive/graphics/slic.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/cuda-spec.hpp"
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

namespace perceive
{
#ifdef WITH_CUDA
CATCH_TEST_CASE("CudaSLIC", "[cuda_slic]")
{
   CATCH_SECTION("slic-labels")
   {
      if(!cuda::cuda_is_available()) {
         WARN("no CUDA device found, skipping CudaSLIC");
         return;
      }

      // A 4x4 grid of flat-coloured blocks: superpixels should not straddle
      // block boundaries, and every pixel must receive a label.
      constexpr unsigned w = 256, h = 192, block = 64;
      ARGBImage argb{w, h};
      for(unsigned y = 0; y < h; ++y)
         for(unsigned x = 0; x < w; ++x) {
            const unsigned k = (y / block) * 4 + (x / block);
            argb(x, y) = make_colour((k * 53) % 256, (k * 97) % 256,
                                     (k * 151) % 256);
         }

      const Vec3fImage lab = cuda::ARGBToLABConversion{}.convert(argb);
      cuda::slic::SLIC slic;
      const auto result = slic.calc_labels(lab, 32 * 32, 10.0f);

      CATCH_REQUIRE(result.labels.width == w);
      CATCH_REQUIRE(result.labels.height == h);
      CATCH_REQUIRE(result.n_labels > 1);

      unsigned n_invalid = 0;
      for(unsigned y = 0; y < h; ++y)
         for(unsigned x = 0; x < w; ++x)
            if(result.labels(x, y) < 0
               || unsigned(result.labels(x, y)) >= result.n_labels)
               ++n_invalid;
      CATCH_REQUIRE(n_invalid == 0);
   }
}
#endif

} // namespace perceive
