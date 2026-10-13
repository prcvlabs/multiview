

#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "perceive/contrib/catch.hpp"

#include "perceive/optimization/kz-filter.hpp"

namespace perceive
{
// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("KzFilter0", "[kz-filter-0]")
{
   CATCH_SECTION("kz-filter-0")
   {
      //      INFO("HEllo");

      // KzFilter filter;
      // filter.init(5, 4);
      // cout << str(filter) << endl;
   }
}

// -------------------------------------------------------------- window and lag
// KZ(m, k) convolves a box filter of width m with itself k - 1 times. The
// tracker smooths with KZ(9, 4): 33 coefficients, half-width 16.
//
CATCH_TEST_CASE("KzFilterWindow", "[kz_filter]")
{
   CATCH_SECTION("kz-filter-9-4")
   {
      const KzFilter filter(9, 4);
      CATCH_REQUIRE(filter.window_size() == 33);
      CATCH_REQUIRE(filter.delta_t() == 16);
      CATCH_REQUIRE(filter.m() == 9);
      CATCH_REQUIRE(filter.k() == 4);
   }

   CATCH_SECTION("kz-filter-5-2")
   {
      const KzFilter filter(5, 2);
      CATCH_REQUIRE(filter.window_size() == 9);
      CATCH_REQUIRE(filter.delta_t() == 4);
      CATCH_REQUIRE(filter.m() == 5);
   }

   CATCH_SECTION("kz-filter-smooths-a-constant")
   {
      // A constant signal is unchanged, also where samples are missing
      const KzFilter filter(9, 4);
      auto get = [](int t) -> std::optional<real> {
         if(t < 0 || t >= 40) return std::nullopt;
         return 2.5;
      };
      for(int t = 0; t < 40; ++t)
         CATCH_REQUIRE(std::fabs(filter.smooth<real>(get, t) - 2.5) < 1e-12);
   }
}

} // namespace perceive
