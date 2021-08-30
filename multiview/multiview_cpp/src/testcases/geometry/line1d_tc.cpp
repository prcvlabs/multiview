
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/geometry/line-1d.hpp"

static const bool feedback = false;

namespace perceive
{
template<typename T> void test_line1d()
{
   using Line1D = Line1DT<T>;
   std::mt19937 g;
   g.seed(0);

   std::uniform_int_distribution<int> dist(-5, 5); // inclusive

   auto test_it = [&]() {
      const auto l0 = Line1D(T(dist(g)), T(dist(g)));
      const auto l1 = l0.normalised();
      const auto r0 = Line1D(T(dist(g)), T(dist(g)));
      const auto r1 = r0.normalised();
      const auto dd = difference_1d(l1, r1);
      const auto ii = intersection_1d(l1, r1);

      CATCH_REQUIRE(l1.a <= l1.b);
      CATCH_REQUIRE(l1.length() >= T(0));

      CATCH_REQUIRE(union_1d(l0, l1) == l1);
      if(l0.length() > 0) CATCH_REQUIRE(intersection_1d(l0, l1) == l1);

      CATCH_REQUIRE(union_1d(l1, r1).length() >= l1.length());
      CATCH_REQUIRE(union_1d(l1, r1).length() >= r1.length());
      CATCH_REQUIRE(intersection_1d(l1, r1).length() <= l1.length());
      CATCH_REQUIRE(intersection_1d(l1, r1).length() <= r1.length());
      CATCH_REQUIRE(intersection_1d(l1, r1).length() >= T(0));
      CATCH_REQUIRE(intersection_1d(l1, r0).length() >= T(0));
      CATCH_REQUIRE(intersection_1d(l0, r1).length() >= T(0));
      CATCH_REQUIRE(intersection_1d(l0, r0).length() >= T(0));

      if(ii.empty()) {
         CATCH_REQUIRE(dd == l1);
      } else {
         if(l1.b <= r1.b)
            CATCH_REQUIRE(dd.length() + ii.length() == l1.length());
         if(r1.a > l1.a) CATCH_REQUIRE(dd.b == r1.a);
      }
   };

   for(auto i = 0; i < 100; ++i) test_it();
}

CATCH_TEST_CASE("Line1D", "[line-1d]")
{
   // ---------------------------------------------------- Merge Track Endpoints
   CATCH_SECTION("line-1d")
   {
      test_line1d<float>();
      test_line1d<double>();
      test_line1d<int>();
   }
}

} // namespace perceive
