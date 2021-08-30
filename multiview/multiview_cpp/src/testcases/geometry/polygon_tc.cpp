
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"

// Tests:
// * Point in polygon
// * Polygon line intersections

namespace perceive
{
static std::vector<Vector2> hull1{{{0., 0.},
                                   {0., 3.},
                                   {2., 2.},
                                   {5., 4.},
                                   {5., 2.},
                                   {3., 4.},
                                   {3., 0.},
                                   {0., 0.}}};

static std::vector<Vector2> chull1{
    {{1., 1.}, {0., 2.}, {0., 3.}, {2., 4.}, {4., 4.}, {4., 1.}}};

static std::vector<Vector2> chull2{
    {{.5, 1.}, {.5, 3.5}, {1., 4.}, {2., 3.5}, {5., 3.5}, {5., .5}}};

static std::vector<Vector2> hull2{
    {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}};

static std::vector<Vector2> hull3{
    {{0.0, 0.0}, {3.0, 0.0}, {3.0, 1.0}, {0.0, 1.0}}};

// ----------------------------------------------------- Point in/out of Polygon

CATCH_TEST_CASE("Point in/out of polygon", "[point_in_polygon]")
{
   CATCH_SECTION("Hull vertices inside hull1")
   {
      for(const auto& X : hull1) {
         CATCH_REQUIRE(point_in_polygon(X, cbegin(hull1), cend(hull1)));
      }
   }

   CATCH_SECTION("Hull edge midpoints: inside hull1")
   {
      auto ii = cbegin(hull1);
      auto jj = ii;
      std::advance(jj, size(hull1) - 1);
      for(; ii != cend(hull1); jj = ii++) {
         auto X = 0.5 * (*ii + *jj);
         CATCH_REQUIRE(point_in_polygon(X, cbegin(hull1), cend(hull1)));
      }
   }

   CATCH_SECTION("Points collinear to hull edge, but outside hull1")
   {
      auto ii = cbegin(hull1);
      auto jj = ii;
      std::advance(jj, size(hull1) - 1);
      for(; ii != cend(hull1); jj = ii++) {
         if(*ii == *jj) continue;
         auto X  = *jj + 10.0 * (*ii - *jj);
         auto in = point_in_polygon(X, cbegin(hull1), cend(hull1));
         CATCH_REQUIRE(!in);
      }
   }

   CATCH_SECTION("Points in hull1")
   {
      CATCH_REQUIRE(
          point_in_polygon(Vector2{1.0, 1.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          point_in_polygon(Vector2{1.0, 2.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          point_in_polygon(Vector2{2.0, 1.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          point_in_polygon(Vector2{4.5, 3.5}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          point_in_polygon(Vector2{3.1, 3.5}, cbegin(hull1), cend(hull1)));
   }

   CATCH_SECTION("Points not in hull1")
   {
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{4.0, 0.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{4.0, 1.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{4.0, 2.5}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{4.0, 4.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{0.0, 4.0}, cbegin(hull1), cend(hull1)));
      CATCH_REQUIRE(
          !point_in_polygon(Vector2{2.0, 3.0}, cbegin(hull1), cend(hull1)));
   }
}

// --------------------------------------------------- Line-Polygon Intersection

CATCH_TEST_CASE("Line-polygon intersection", "[hull_line_intersections]")
{
   vector<Vector2> isects;
   auto calc_isects = [&](const Vector2& a, const Vector2& b) -> auto&
   {
      auto l = to_homgen_line(a, b);
      hull_line_intersections(cbegin(hull1), cend(hull1), l, isects);
      for(auto& X : isects)
         CATCH_REQUIRE(point_in_polygon(X, cbegin(hull1), cend(hull1)));
      return isects;
   };

   CATCH_SECTION("No line intersections")
   {
      CATCH_REQUIRE(calc_isects(Vector2{0., -1.0}, Vector2{3., -1.0}).size()
                    == 0);
      CATCH_REQUIRE(calc_isects(Vector2{6.0, 0.0}, Vector2{6.0, 1.0}).size()
                    == 0);
      CATCH_REQUIRE(calc_isects(Vector2{0., -1.0}, Vector2{-1.0, 0.0}).size()
                    == 0);
   }

   CATCH_SECTION("Intersecting a single vertex")
   {
      CATCH_REQUIRE(calc_isects(Vector2{-1., 1.0}, Vector2{1.0, -1.}).size()
                    == 1);
      CATCH_REQUIRE(calc_isects(Vector2{-1., 2.0}, Vector2{1.0, 4.0}).size()
                    == 1);
   }

   CATCH_SECTION("Intserecting a hull edge")
   {
      CATCH_REQUIRE(calc_isects(Vector2{0.0, 0.0}, Vector2{0.0, 4.0}).size()
                    > 0);
      CATCH_REQUIRE(calc_isects(Vector2{2.0, 2.0}, Vector2{5.0, 4.0}).size()
                    > 0);
   }

   CATCH_SECTION("Two intersections")
   {
      CATCH_REQUIRE(calc_isects(Vector2{0.0, 1.0}, Vector2{1.0, 1.0}).size()
                    == 2);
      CATCH_REQUIRE(calc_isects(Vector2{1.0, 0.0}, Vector2{2.0, 0.0}).size()
                    == 2);
      CATCH_REQUIRE(calc_isects(Vector2{0.0, 1.0}, Vector2{1.0, 0.0}).size()
                    == 2);
   }

   CATCH_SECTION("Four intersections")
   {
      CATCH_REQUIRE(calc_isects(Vector2{0.0, 3.5}, Vector2{1.0, 3.5}).size()
                    == 4);
   }
}

// ----------------------------------------------------- Point in/out of Polygon

CATCH_TEST_CASE("Inscribed rectangle", "[inscribed_AABB_two_hulls]")
{
   auto find_bounding_roi = [&](const auto& hull) -> AABB {
      // Find the bounding rectangle
      auto aabb = AABB::minmax();
      for(const auto& X : hull) aabb.union_point(X);
      return aabb;
   };

   array<vector<Vector2>, 2> hull{{chull1, chull2}};
   array<AABB, 2> bounds;
   std::transform(cbegin(hull), cend(hull), begin(bounds), find_bounding_roi);

   // What's the average center of the two hulls?
   array<vector<Vector2>, 2> isects;

   auto hull_hull_line_bounds = [&](const real y) {
      auto l = to_homgen_line(Vector2{0.0, y}, Vector2{1.0, y});
      for(size_t i = 0; i < 2; ++i) {
         hull_line_intersections(cbegin(hull[i]), cend(hull[i]), l, isects[i]);
         if(isects[i].size() < 2) return Vector2::nan();
      }
      auto cmp = [&](const auto& a, const auto& b) { return a.x < b.x; };
      auto [l0, r0]
          = std::minmax_element(cbegin(isects[0]), cend(isects[0]), cmp);
      auto [l1, r1]
          = std::minmax_element(cbegin(isects[1]), cend(isects[1]), cmp);
      return Vector2(std::max(l0->x, l1->x), std::min(r0->x, r1->x));
   };

   auto calc_aabb = [&](const real y0, const real y1) -> AABB {
      auto xx0 = hull_hull_line_bounds(y0);
      auto xx1 = hull_hull_line_bounds(y1);
      AABB aabb;
      aabb.left                       = std::max(xx0(0), xx1(0));
      aabb.right                      = std::min(xx0(1), xx1(1));
      std::tie(aabb.top, aabb.bottom) = std::minmax(y0, y1);
      if(!aabb.is_finite()) aabb = AABB(0.0, 0.0, 0.0, 0.0);
      return aabb;
   };

   auto test_aabb = [&](const real y0, const real y1, const AABB& expected) {
      auto aabb = calc_aabb(y0, y1);
      auto ia   = intersection_area(aabb, expected);
      auto ua   = union_area(aabb, expected);
      return (ia / ua) - 1.0 < 1e-9;
   };

   CATCH_SECTION("Inscribed-Rectangle")
   {
      CATCH_REQUIRE(test_aabb(3.0, 2.0, AABB(0.5, 2.0, 4.0, 3.0)));
      CATCH_REQUIRE(test_aabb(2.0, 3.0, AABB(0.5, 2.0, 4.0, 3.0)));
      CATCH_REQUIRE(test_aabb(1.0, 3.5, AABB(1.0, 1.0, 4.0, 3.5)));
      CATCH_REQUIRE(test_aabb(1.1, 3.6, AABB(1.2, 1.1, 1.8, 3.6)));
      CATCH_REQUIRE(calc_aabb(0.0, 2.0).area() == 0.0);
      CATCH_REQUIRE(calc_aabb(2.0, 2.0).area() == 0.0);
   }
}

// ----------------------------------------------------- Point in/out of Polygon

CATCH_TEST_CASE("Union area, two convex hull", "[union-area-two-convex-hulls]")
{
   auto i1 = convex_hull_intersection(
       cbegin(hull2), cend(hull2), cbegin(hull3), cend(hull3));
   CATCH_SECTION("union-two-convex-hulls")
   {
      CATCH_REQUIRE(fabs(polygon_area(cbegin(i1), cend(i1)) - 2.0) < 1e-9);
   }
}

// ------------------------------------------------------ Line/AABB intersection

CATCH_TEST_CASE("Line/AABB intersection", "[line-aabb-intersection]")
{
   CATCH_SECTION("line-aabb-intersection")
   {
      auto test_it = [&](const AABB aabb,
                         const Vector2 a,
                         const Vector2 b,
                         const bool does_isect) {
         const auto [u, v] = intersect(aabb, a, b);
         if(false) {
            const auto s = "AABB = {:s}, ({{}, {}} -> {{}, {}}) => "
                           "({{}, {}} -> {{}, {}})";
            INFO(format(s, str(aabb), a.x, a.y, b.x, b.y, u.x, u.y, v.x, v.y));
         }

         if(does_isect) {
            CATCH_REQUIRE(u.is_finite());
            CATCH_REQUIRE(v.is_finite());
            CATCH_REQUIRE(aabb.contains(u.round()));
            CATCH_REQUIRE(aabb.contains(v.round()));
         } else {
            CATCH_REQUIRE(!u.is_finite());
            CATCH_REQUIRE(!v.is_finite());
         }
      };

      // Both points in bounds
      test_it(AABB(0, 0, 100, 50), {10.0, 10.0}, {20.0, 20.0}, true);

      // One point in bounds
      test_it(AABB(0, 0, 100, 50), {-10.0, 10.0}, {20.0, 10.0}, true);
      test_it(AABB(0, 0, 100, 50), {10.0, 10.0}, {20.0, 120.0}, true);
      test_it(AABB(0, 0, 100, 50), {-10.0, 30.0}, {110.0, 30.0}, true);
      test_it(AABB(0, 0, 100, 50), {-10.0, -10.0}, {-10.0, -20.0}, false);

      // Line along the bounds
      test_it(AABB(0, 0, 100, 50), {0.0, 10.0}, {0.0, 20.0}, true); // left
      test_it(AABB(0, 0, 100, 50), {0.0, -10.0}, {0.0, 20.0}, true);
      test_it(AABB(0, 0, 100, 50), {0.0, 10.0}, {0.0, 200.0}, true);
      test_it(AABB(0, 0, 100, 50), {0.0, -10.0}, {0.0, 200.0}, true);
      test_it(AABB(0, 0, 100, 50), {10.0, 0.0}, {20.0, 0.0}, true); // top
      test_it(AABB(0, 0, 100, 50), {-10.0, 0.0}, {20.0, 0.0}, true);
      test_it(AABB(0, 0, 100, 50), {10.0, 0.0}, {200.0, 0.0}, true);
      test_it(AABB(0, 0, 100, 50), {-10.0, 0.0}, {200.0, 0.0}, true);
      test_it(AABB(0, 0, 100, 50), {100.0, 10.0}, {100.0, 20.0}, true); // r
      test_it(AABB(0, 0, 100, 50), {100.0, -10.0}, {100.0, 20.0}, true);
      test_it(AABB(0, 0, 100, 50), {100.0, 10.0}, {100.0, 200.0}, true);
      test_it(AABB(0, 0, 100, 50), {100.0, -10.0}, {100.0, 200.0}, true);
      test_it(AABB(0, 0, 100, 50), {10.0, 50.0}, {20.0, 50.0}, true); // b
      test_it(AABB(0, 0, 100, 50), {-10.0, 50.0}, {20.0, 50.0}, true);
      test_it(AABB(0, 0, 100, 50), {10.0, 50.0}, {200.0, 50.0}, true);
      test_it(AABB(0, 0, 100, 50), {-10.0, 50.0}, {200.0, 50.0}, true);

      test_it(
          AABB(0, 0, 896, 672), {1078.77, 408.555}, {1097.73, 430.008}, false);

      CATCH_REQUIRE(true);
   }
}

// [(1078.77, 408.555) -> (1097.73, 430.008)]  ==>  [(717.696, -0) -> (896,
// 201.751)], bounds = [0 0 896 672]

} // namespace perceive
