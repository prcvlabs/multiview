
#include <algorithm>
#include <iterator>

#include "perceive/graphics/colour-set.hpp"

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

namespace perceive
{
// ----------------------------------------------------------------

CATCH_TEST_CASE("ColourSet", "[colour_set]")
{
   //
   // ------------------------------------------------------- FowlkesGraph_3332
   //
   CATCH_SECTION("rgb2lab")
   {
      auto test_colour = [&](const auto k0) {
         Vector3 rgb  = kolour_to_vector3(k0);
         Vector3 lab  = rgb_to_lab(rgb);
         Vector3 out  = lab_to_rgb(lab);
         Vector3 lab1 = rgb_to_lab(out);
         uint32_t k1  = vector3_to_kolour(out);

         CATCH_REQUIRE((lab - lab1).norm() < 1.0);
         // cout << format("k = {:8x}|{:8x}, rgb = {{:s}, {:s}}, dist={}",
         //                k0, k1,
         //                str(rgb), str(out),
         //                (lab-lab1).norm()) << endl;
      };

      for(auto i = 0u; i < n_crayons(); ++i) test_colour(colour_set_1(i));
   }
}

} // namespace perceive
