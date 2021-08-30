
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/integral-image.hpp"

using namespace perceive;

CATCH_TEST_CASE("IntegralImage", "[integral_image]")
{
   CATCH_SECTION("TestIntegralImage")
   {
      FloatImage f;
      const auto w = 5;
      const auto h = 3;

      f.resize(w, h);

      auto a = 0;
      auto b = 0;

      for(auto y = 0u; y < h; ++y) {
         for(auto x = 0u; x < w; ++x) {
            auto c  = (b == 0) ? 1 : a + b;
            f(x, y) = float(c);
            a       = b;
            b       = c;
         }
      }

      // Make the integral images
      vector<float> i_f(w * h);
      make_integral_image(f.pixels, w, h, &i_f[0]);

      for(auto t = 0; t < h; ++t) {
         for(auto b = t; b < h; ++b) {
            for(auto l = 0; l < w; ++l) {
               for(auto r = l; r < w; ++r) {
                  auto sum = sum_region(f.pixels, w, h, l, t, r, b);
                  auto iii = integral_region(&i_f[0], w, h, l, t, r, b);

                  CATCH_REQUIRE(std::abs(sum - iii) < 1e-9f);
               }
            }
         }
      }
   }
}
