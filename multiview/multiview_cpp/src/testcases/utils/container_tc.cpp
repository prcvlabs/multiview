
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/io/perceive-assets.hpp"

namespace perceive
{
CATCH_TEST_CASE("ContainerStuff", "[container_stuff]")
{
   CATCH_SECTION("test-last-if")
   {
      const vector<int> Xs = {0, 0, 0, 1, 1, 1, 1};
      const vector<int> Ys = {1, 2, 3, 4, 5, 6, 7, 8, 9};
      const vector<int> Zs = {};

      auto is_mod_3 = [](int x) { return modulo(x, 3) == 0; };

      {
         auto ii = find_last_if(
             cbegin(Xs), cend(Xs), [&](auto o) { return o == 0; });
         CATCH_REQUIRE(ii == cbegin(Xs) + 2);
      }
      {
         auto ii = find_last_if(
             crbegin(Xs), crend(Xs), [&](auto o) { return o == 0; });
         CATCH_REQUIRE(ii == crbegin(Xs) + long(Xs.size()) - 1);
      }
      {
         auto ii = find_last_if(
             cbegin(Xs), cend(Xs), [](auto o) { return o == 2; });
         CATCH_REQUIRE(ii == cend(Xs));
      }
      {
         auto ii = find_last_if(
             crbegin(Xs), crend(Xs), [](auto o) { return o == 2; });
         CATCH_REQUIRE(ii == crend(Xs));
      }
      {
         auto ii = find_last_if(cbegin(Ys), cend(Ys), is_mod_3);
         CATCH_REQUIRE(*ii == 9);
      }
      {
         auto ii = find_last_if(crbegin(Ys), crend(Ys), is_mod_3);
         CATCH_REQUIRE(*ii == 3);
      }
      {
         auto ii = find_last_if(crbegin(Zs), crend(Zs), is_mod_3);
         CATCH_REQUIRE(ii == crend(Zs));
      }
   }

   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("make-generator-function")
   {
      vector<int> v  = {0, 1, 2, 3, 4};
      auto next      = make_generator_function(cbegin(v), cend(v));
      const int* ptr = nullptr;
      auto counter   = 0u;
      for(auto ptr = next(); ptr != nullptr; ptr = next())
         CATCH_REQUIRE(*ptr == v[counter++]);

      CATCH_REQUIRE(counter == v.size());
   }

   CATCH_SECTION("smallest-elem")
   {
      array<real, 10> Xs;
      std::iota(begin(Xs), end(Xs), 0);

      {
         auto ii = smallest_elem(cbegin(Xs), cbegin(Xs));
         CATCH_REQUIRE(ii == cbegin(Xs));
      }

      {
         auto ii = smallest_elem(std::next(cbegin(Xs)), std::next(cbegin(Xs)));
         CATCH_REQUIRE(ii == std::next(cbegin(Xs)));
      }

      {
         auto ii = smallest_elem(cbegin(Xs), cend(Xs));
         CATCH_REQUIRE(ii == cbegin(Xs));
      }

      {
         auto ii = smallest_elem(cbegin(Xs), cend(Xs), std::greater<>());
         CATCH_REQUIRE(ii == std::next(cbegin(Xs), Xs.size() - 1));
      }
   }
}

} // namespace perceive
