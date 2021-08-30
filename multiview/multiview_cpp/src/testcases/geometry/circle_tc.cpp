
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/circle.hpp"
#include "perceive/geometry/polygon.hpp"

namespace perceive
{
CATCH_TEST_CASE("CircleIU", "[circle-iu]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("circle-iu")
   {
      const Vector2 u = Vector2{1.0, 1.0};
      const auto r    = 1.0;
      for(auto d = 0.0; d <= 2.2; d += 0.2) {
         const Vector2 v = u + d * Vector2{0.0, 1.0};
         const auto A    = circles_intersection_area(u, v, r);
         const auto f    = 6.0 * A / M_PI;
         // cout << format(" d = {}, f = {}", d, f) << endl;
      }
   }

   CATCH_SECTION("circle-iu2")
   {
      // U is inside V
      const auto A    = Vector2{0.0, 0.0};
      const auto B    = Vector2{1.0, 0.0};
      const auto iuAB = intersection_area(A, 1.0, B, 20.0);
      const auto iuBA = intersection_area(A, 20.0, B, 1.0);
      CATCH_REQUIRE(is_close(iuAB, M_PI));
      CATCH_REQUIRE(is_close(iuBA, M_PI));
      CATCH_REQUIRE(is_close(intersection_area(A, 0.5, B, 0.5), 0.0));
   }
}

} // namespace perceive
