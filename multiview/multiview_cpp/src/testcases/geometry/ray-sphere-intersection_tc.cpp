
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/ray-sphere-intersection.hpp"
#include "perceive/utils/math.hpp"

namespace perceive
{
CATCH_TEST_CASE("RaySphereIntersection", "[ray-sphere-intersection]")
{
   CATCH_SECTION("ray-sphere-intersection")
   {
      const Vector3f C   = {1.0f, 2.0f, 3.0f};
      const Vector3f ray = -C.normalised();
      const Vector3f S   = {-0.1f, 0.4f, 1.0f};
      const float radius = 2.0f;

      const auto [k0, k1] = ray_sphere_intersection(C, ray, S, radius);

      const auto U = C + k0 * ray;
      const auto V = C + k1 * ray;

      CATCH_REQUIRE(is_close((U - S).norm(), radius));
      CATCH_REQUIRE(is_close((V - S).norm(), radius));
   }
}

} // namespace perceive
