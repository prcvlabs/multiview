
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/math.hpp"

namespace perceive
{
CATCH_TEST_CASE("PlaneP3", "[plane-p3]")
{
   CATCH_SECTION("plane-p3")
   {
      auto test_it = [&](const Plane& A, const Plane& B, const Plane& C) {
         const auto O = intersection_of_3_planes(A, B, C);
         CATCH_REQUIRE(std::fabs(A.side(O)) < 1e-3);
         CATCH_REQUIRE(std::fabs(B.side(O)) < 1e-3);
         CATCH_REQUIRE(std::fabs(C.side(O)) < 1e-3);
      };

      test_it({1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0});
      test_it({1.0, 0.0, 0.0, 1.0}, {1.0, 1.0, 0.0, 2.0}, {0.0, 1.0, 1.0, 3.0});
   }
}

} // namespace perceive
