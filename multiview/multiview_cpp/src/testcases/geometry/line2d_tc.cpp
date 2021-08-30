
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/geometry/line-2d.hpp"

static const bool feedback = false;

namespace perceive
{
template<typename T>
static void test_abcd(const Vector2T<T>& a,
                      const Vector2T<T>& b,
                      const Vector2T<T>& c,
                      const Vector2T<T>& d,
                      const bool is_true) noexcept
{
   constexpr auto ep = T(1e-9);
   const auto A      = Vector3T<T>{a(0), a(1), T(1.0)};
   const auto B      = Vector3T<T>{b(0), b(1), T(1.0)};
   const auto C      = Vector3T<T>{c(0), c(1), T(1.0)};
   const auto D      = Vector3T<T>{d(0), d(1), T(1.0)};
   const auto E      = segment_segment_intersection(a, b, c, d);

   if(is_true) {
      CATCH_REQUIRE(dist_to_segment_P2(A, B, E) < ep);
      CATCH_REQUIRE(dist_to_segment_P2(C, D, E) < ep);
   } else {
      CATCH_REQUIRE(!E.is_finite());
   }
}

template<typename T>
static void test_abcdz(const Vector2T<T>& a,
                       const Vector2T<T>& b,
                       const Vector2T<T>& c,
                       const Vector2T<T>& d,
                       const bool is_true) noexcept
{
   test_abcd(a, b, c, d, is_true);
   test_abcd(a, b, d, c, is_true);
   test_abcd(b, a, c, d, is_true);
   test_abcd(b, a, d, c, is_true);
}

CATCH_TEST_CASE("Line2D", "[line-2d]")
{
   // ---------------------------------------------------- Merge Track Endpoints
   CATCH_SECTION("line-2d")
   {
      Vector2 a{1.0, 2.0};
      Vector2 b{2.0, 2.0};
      Vector2 c{1.0, 1.0};
      Vector2 d{2.0, 1.0};
      Vector2 e{1.0, 0.0};

      test_abcdz(a, d, b, c, true);
      test_abcdz(a, e, b, d, false);
      test_abcdz(a, c, d, e, false);
      test_abcdz(a, c, b, e, false);
      test_abcdz(a, c, b, c, true);
      test_abcdz(a, c, c, b, true);
      test_abcdz(c, b, e, d, false);
   }
}

} // namespace perceive
