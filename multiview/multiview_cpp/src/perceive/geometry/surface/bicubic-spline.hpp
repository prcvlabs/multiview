
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
class BicubicSpline
{
 private:
   AABB bounds_;
   real dx_{0.0}; // granularity in x-direction, derived
   real dy_{0.0}; // granularity in y-direction
   real dx_inv_{0.0};
   real dy_inv_{0.0};

   // [f(x), f_x(x), f_y(x), f_xy(x)]
   ImageContainerT<Vector4r> grid_;
   ImageContainerT<Matrix4r> A_;

 public:
   CUSTOM_NEW_DELETE(BicubicSpline)

   void init(const AABB& bounds, unsigned n_cols, unsigned n_rows);
   void finalize(); // MUST CALL THIS after modifying the grid

   real evaluate(Vector2 xy) const { return evaluate(xy.x, xy.y); }
   real evaluate(real x, real y) const;

   const decltype(grid_)& grid() const { return grid_; }
   const AABB& bounds() const { return bounds_; }
   real dx() const { return dx_; }
   real dy() const { return dy_; }

   unsigned width() const { return grid().width; }
   unsigned height() const { return grid().height; }

   Vector2 position(Point2 p) const { return position(p.x, p.y); }
   Vector2 position(int x, int y) const
   {
      return Vector2(x * dx() + bounds_.left, y * dy() + bounds_.top);
   }
   bool in_bounds(Point2 p) const { return grid().in_bounds(p); }
   bool in_bounds(int x, int y) const { return grid().in_bounds(x, y); }
   bool in_aabb(Vector2 x) const { return bounds().contains(x); }
   bool in_aabb(real x, real y) const { return bounds().contains(x, y); }

   // Returns the top-left grid coordinates relevant for evaluating 'x'
   Point2 top_left(real x, real y) const;
   Point2 top_left(const Vector2& x) const { return top_left(x.x, x.y); }
   Vector4r& X(int x, int y) { return grid_(x, y); }
   const Vector4r& X(int x, int y) const { return grid_(x, y); }

   std::string to_string() const;
   // unsigned n_params() const { return 4 + grid_.size() * 4; }
   // void write(real *) const;
   // void read(const real *); // init must have been called with correct dims

   void serialize(std::ostream& out) const;
   void unserialize(std::istream& in);
};

} // namespace perceive
