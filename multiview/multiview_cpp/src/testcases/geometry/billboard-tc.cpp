
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/utils/math.hpp"

namespace perceive
{
CATCH_TEST_CASE("billboard", "[billboard]")
{
   std::mt19937 gen;
   std::uniform_real_distribution<double> rand_theta{-M_PI, M_PI};
   std::uniform_real_distribution<double> rand1{0.0, 1000.0};
   gen.seed(0);

   auto random_n = [&]() {
      return spherical_to_cartesian(
          Vector3(rand_theta(gen), rand_theta(gen), 1.0));
   };

   auto random_p3 = [&]() { return Plane(random_n(), rand_theta(gen)); };

   auto make_2d_polygon = [&]() {
      vector<Vector2> out;
      for(size_t i = 0; i < 8; ++i) out[i] = Vector2(rand1(gen), rand1(gen));
      return out;
   };

   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("p3-relative")
   {
      gen.seed(0);
      auto run_test = [&]() {
         const auto p3 = random_p3();
         const auto C
             = Vector3(rand_theta(gen), rand_theta(gen), rand_theta(gen));
         auto gen_n = [&]() {
            auto n = random_n();
            while(std::fabs(dot(n, p3.xyz())) < 0.01) n = random_n();
            return n;
         };

         const auto n = gen_n();

         for(auto d = -1.0; d <= 1.0; d += 0.1) {
            const auto A = ray_position_relative_to_plane(p3, C, C + n, d);
            const auto B = plane_ray_intersection(p3, C, C + n);

            // We're the correct distance
            CATCH_REQUIRE(std::fabs(p3.side(A) - d) < 1e-9);
            CATCH_REQUIRE(angle_ABC2(C, C + n, A) < 1e-3);

            CATCH_REQUIRE(std::fabs(p3.side(B)) < 1e-9);
            CATCH_REQUIRE(angle_ABC2(C, C + n, B) < 1e-3);
         }
      };

      for(auto i = 0; i < 1000; ++i) run_test();
   }
}

} // namespace perceive
