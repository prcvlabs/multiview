
#include <algorithm>
#include <iterator>

#include "perceive/graphics/LAB.hpp"
#include "perceive/graphics/colour-set.hpp"

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

namespace perceive
{
// ----------------------------------------------------------------

CATCH_TEST_CASE("LABColor", "[lab_color]")
{
   //
   // ------------------------------------------------------- FowlkesGraph_3332
   //
   CATCH_SECTION("lab-color")
   {
      Vector3 mins;
      Vector3 maxs;
      for(auto i = 0; i < 3; ++i) {
         mins[i] = std::numeric_limits<real>::max();
         maxs[i] = std::numeric_limits<real>::lowest();
      }

      vector<float> dists;

      float min_dist = std::numeric_limits<float>::max();
      float max_dist = std::numeric_limits<float>::lowest();

      auto test_colour = [&](const uint32_t k0) {
         LAB lab0    = kolour_to_LAB(k0);
         Vector3 rgb = LAB_to_rgb(lab0);
         LAB lab1    = rgb_to_LAB(rgb);

         const auto dist = cie1976_distance(lab0, lab1);
         if(dist < min_dist) min_dist = dist;
         if(dist > max_dist) max_dist = dist;
         CATCH_REQUIRE(dist < 1.0f);
      };

      for(auto i = 0u; i < n_crayons(); ++i) test_colour(colour_set_1(i));

      // for(auto i = 0u; i < n_crayons(); ++i)
      //    for(auto j = i + 1; j < n_crayons(); ++j)
      //       dists.push_back(cie1976_distance(kolour_to_LAB(colour_set_1(i)),
      //                                        kolour_to_LAB(colour_set_1(j))));

      // cout << format("mins = {}", str(mins)) << endl;
      // cout << format("maxs = {}", str(maxs)) << endl;

      // cout << format("minmax = {}, {}", min_dist, max_dist) << endl;

      // cout << str(calc_sample_statistics(begin(dists), end(dists))) << endl;
   }
}

} // namespace perceive
