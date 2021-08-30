
#include "draw-floor-grid.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/graphics/tiny-string.hpp"

namespace perceive
{
// ------------------------------------------------------------- draw floor grid

void draw_floor_grid_in_situ(ARGBImage& distorted,
                             const AABB aabb,
                             const CachingUndistortInverse& cu,
                             const EuclideanTransform& et) noexcept
{
   if(aabb.area() <= 0.0 or !aabb.is_finite()) {
      WARN(format("bounds was empty or otherwise invalid!"));
      return;
   }

   ARGBImage& out = distorted;

   const auto im_bounds = distorted.bounds();
   const auto et_inv    = et.inverse();

   auto round_it
       = [&](const real x) -> real { return std::round(x * 10.0) * 0.1; };

   auto project = [&](const Vector3& Y) -> Vector2 {
      return cu.distort(homgen_P2_to_R2(et_inv.apply(Y)));
   };

   real l = round_it(aabb.left);
   real r = round_it(aabb.right);
   if(l > r) std::swap(l, r);

   real t = round_it(aabb.top);
   real b = round_it(aabb.bottom);
   if(t > b) std::swap(t, b);

   // INFO(format("bounds = {}", str(aabb)));
   // cout << format("l = {} -> r = {}", l, r) << endl;
   // cout << format("t = {} -> b = {}", t, b) << endl;

   auto is_solid = [&](const real x) -> bool {
      return std::fabs(x - std::round(x)) < 1e-6;
   };

   auto draw_segment =
       [&](const Vector2& x0, const Vector2& x1, bool thick, uint32_t kolour) {
          // Project
          const auto X = project(Vector3(x0.x, x0.y, 0.0));
          const auto Y = project(Vector3(x1.x, x1.y, 0.0));

          if(!out.in_bounds(X) or !out.in_bounds(Y)) { return; }

          if(thick) {
             plot_line_AA(X, Y, im_bounds, [&](int x, int y, float a) {
                if(out.in_bounds(x, y)) out(x, y) = blend(kolour, out(x, y), a);
             });
          } else {
             int counter = 0;
             bresenham(X, Y, im_bounds, [&](int x, int y) {
                if(!out.in_bounds(x, y)) return;
                if((++counter / 4) % 2)
                   out(x, y) = blend(kolour, out(x, y), 0.5);
             });
          }
       };

   auto draw_line = [&](const Vector2& x0, const Vector2& x1, bool thick) {
      auto is_zero
          = [](const real x) { return std::fabs(std::round(x * 10.0)) < 1e-6; };
      constexpr real e     = 1e-6;
      const bool is_y_axis = is_zero(x0.x) && is_zero(x1.x)
                             && inclusive_between(0.0 - e, x0.y, 1.0 - e);
      const bool is_x_axis = is_zero(x0.y) && is_zero(x1.y)
                             && inclusive_between(0.0 - e, x0.x, 1.0 - e);
      const auto kolour = is_x_axis ? k_red : is_y_axis ? k_green : k_yellow;
      draw_segment(x0, x1, thick, kolour);
   };

   auto draw_cell = [&](const Vector2& tl, const Vector2& br, bool do_tl) {
      const auto l = tl.x;
      const auto r = br.x;
      const auto t = tl.y;
      const auto b = br.y;
      if(do_tl) {
         draw_line(Vector2(l, t), Vector2(r, t), is_solid(t));
         draw_line(Vector2(l, t), Vector2(l, b), is_solid(l));
      } else {
         draw_line(Vector2(l, b), Vector2(r, b), is_solid(b));
         draw_line(Vector2(r, t), Vector2(r, b), is_solid(r));
      }
   };

   auto draw_corner = [&](const Vector2& x) {
      const auto X = project(Vector3(x.x, x.y, 0.0));
      if(out.in_bounds(X)) {
         const auto off = Point2(4, 4);
         out(X)         = k_red;
         render_string(out,
                       format("{}, {}", std::round(x.x), std::round(x.y)),
                       to_pt2(X) + off,
                       k_cyan,
                       k_black);
      }
   };

   {
      for(auto y = t; y < b; y += 0.1) {
         const bool last_y = std::fabs(b - (y + 0.1)) < 1e-6;
         for(auto x = l; x < r; x += 0.1) {
            const bool last_x = std::fabs(r - (x + 0.1)) < 1e-6;
            const auto tl     = Vector2{x, y};
            const auto br     = tl + Vector2{0.1, 0.1};
            draw_cell(tl, br, true);
            if(last_y && last_x) draw_cell(tl, br, false);
         }
      }
   }

   // Draw intersections
   for(auto y = t; y < b + 0.05; y += 0.1)
      if(is_solid(y))
         for(auto x = l; x < r + 0.05; x += 0.1)
            if(is_solid(x)) draw_corner(Vector2(x, y));
}

ARGBImage draw_floor_grid(const ARGBImage& distorted,
                          const AABB aabb,
                          const CachingUndistortInverse& cu,
                          const EuclideanTransform& et) noexcept(false)
{
   ARGBImage out = distorted;
   draw_floor_grid_in_situ(out, aabb, cu, et);
   return out;
}

void draw_floor_cell_in_situ(ARGBImage& distorted,
                             const AABB cell, // the cell to draw
                             const CachingUndistortInverse& cu,
                             const EuclideanTransform& et,
                             const uint32_t fg_kolour,
                             const uint32_t bg_kolour) noexcept
{
   auto& out            = distorted;
   const auto im_bounds = distorted.bounds();

   std::array<Vector2, 4> Xs; // This is what we're flood-filling
   {
      const auto et_inv = et.inverse();
      auto project      = [&](const Vector2& x) -> Vector2 {
         return cu.distort(
             homgen_P2_to_R2(et_inv.apply(Vector3(x.x, x.y, 0.0))));
      };
      size_t pos = 0;
      for(const Vector2& x : cell.to_array_polygon()) {
         Xs[pos++] = project(x);
      }
   }

   AABBi bounds = aabb_to_aabbi(AABB{cbegin(Xs), cend(Xs)});
   Expects(bounds.left <= bounds.right);
   Expects(bounds.top <= bounds.bottom);

   for(auto y = bounds.top; y <= bounds.bottom; ++y)
      for(auto x = bounds.left; x <= bounds.right; ++x)
         if(distorted.in_bounds(x, y)
            && point_in_polygon(Vector2(x, y), cbegin(Xs), cend(Xs)))
            distorted(x, y) = bg_kolour;

   for(auto i = 0u; i < Xs.size(); ++i) {
      const auto& A = Xs[(i + 0) % Xs.size()];
      const auto& B = Xs[(i + 1) % Xs.size()];
      plot_line_AA(A, B, im_bounds, [&](int x, int y, float a) {
         if(out.in_bounds(x, y)) out(x, y) = blend(fg_kolour, out(x, y), a);
      });
   }
}
} // namespace perceive
