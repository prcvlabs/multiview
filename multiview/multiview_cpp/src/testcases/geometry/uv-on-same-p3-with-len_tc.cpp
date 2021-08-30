
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/projective/UV-on-same-p3-with-len.hpp"
#include "perceive/geometry/rotation.hpp"

namespace perceive
{
CATCH_TEST_CASE("UVOnSameP3WithLen", "[uv-on-same-p3-with-len]")
{
   CATCH_SECTION("uv-on-same-p3-with-len")
   {
      const auto C = Vector3{-1.0, 0.0, -2.0};
      const auto n = Vector3{1.0, 1.1, -2.3}.normalised();

      const auto U = Vector3{1.0, 2.0, 1.1};
      const auto u = (U - C).normalised();

      const auto p3 = Plane(n, -dot(U, n));

      const auto V = plane_ray_intersection(p3, C, C + n);
      const auto v = -(V - C).normalised();

      const auto k_u_ref = (U - C).norm();
      const auto k_v_ref = -(V - C).norm();

      const auto l = (U - V).norm();

      CATCH_REQUIRE(std::fabs(p3.side(U)) < 1e-6);
      CATCH_REQUIRE(std::fabs(p3.side(V)) < 1e-6);
      CATCH_REQUIRE((U - (C + k_u_ref * u)).norm() < 1e-6);
      CATCH_REQUIRE((V - (C + k_v_ref * v)).norm() < 1e-6);

      using T = real;

      const T cos_psi   = std::clamp<T>(dot(u, v), -T(1.0), T(1.0));
      const T cos_theta = std::clamp<T>(dot(u, n), -T(1.0), T(1.0));
      const T cos_phi   = std::clamp<T>(dot(v, n), -T(1.0), T(1.0));
      const T ratio     = cos_theta / cos_phi;

      CATCH_REQUIRE(std::fabs(k_u_ref * cos_theta - k_v_ref * cos_phi) < 1e-6);
      CATCH_REQUIRE(std::fabs(k_v_ref - k_u_ref * ratio) < 1e-6);

      const auto [k_u, k_v] = UV_on_same_p3_with_len(u, v, n, l);

      CATCH_REQUIRE(std::fabs(dot(U, n) - dot(V, n)) < 1e-6); // same plane
      CATCH_REQUIRE(is_close((U - V).norm(), l)); // length is correct

      // cout << format("|U - V| = {}", (U - V).norm()) << endl;
      // cout << format("dot(U, n) = {}", dot(U, n)) << endl;
      // cout << format("dot(V, n) = {}", dot(V, n)) << endl;
   }
}

} // namespace perceive
