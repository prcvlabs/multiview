
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/projective/V-from-U-and-UV-len.hpp"
#include "perceive/geometry/rotation.hpp"

namespace perceive
{
CATCH_TEST_CASE("VFromUAndUVLen", "[v-from-u-and-uv-len]")
{
   CATCH_SECTION("v-from-u-and-uv-len")
   {
      const auto C = Vector3{-1.0, 0.4, -2};
      const auto l = 2.0;
      const auto n = Vector3{1.0, 1.1, -2.3}.normalised();
      const auto U = Vector3{1.0, 2.0, 1.1};
      const auto V = U + l * n;

      const auto u = (U - C).normalised();
      const auto v = (V - C).normalised();

      const auto k_u = (U - C).norm();
      const auto k_v = (V - C).norm();

      const auto ss = square(k_u) - 2 * k_u * k_v * dot(u, v) + square(k_v);
      CATCH_REQUIRE(is_close(ss, square(l)));

      const auto [A, B] = V_from_U_and_UV_len(C, u, v, k_u, l);

      const auto a = (A - C).normalised();
      const auto b = (B - C).normalised();
      CATCH_REQUIRE(is_close(dot(a, b), 1.0)); // colinear

      CATCH_REQUIRE(((V - A).norm() < 1e-3 || (V - B).norm() < 1e-3));

      if(false) {
         INFO(format("REPORT"));
         cout << format("{}^2 - 2 {} {} {} + {}^2 = {}",
                        k_u,
                        k_u,
                        k_v,
                        dot(u, v),
                        k_v,
                        ss)
              << endl;
         cout << format("l^2 = {}", square(l)) << endl;
         cout << vec_feedback(V, A) << endl;
         cout << vec_feedback(V, B) << endl;
      }
   }
}

} // namespace perceive
