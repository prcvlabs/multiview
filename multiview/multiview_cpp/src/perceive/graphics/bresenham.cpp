
#include "bresenham.hpp"

namespace perceive
{
// ------------------------------------------------------------------- bresenham
//
void bresenham(double x1,
               double y1,
               double x2,
               double y2,
               std::function<void(int, int)> func)
{
   using std::swap;

   if(!(std::isfinite(x1) and std::isfinite(y1) and std::isfinite(x2)
        and std::isfinite(y2)))
      return;

   // Bresenham's line algorithm -- adapted from
   // http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C.2B.2B
   const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
   if(steep) {
      swap(x1, y1);
      swap(x2, y2);
   }

   if(false) {
      // Instead of the standard bresenham algorithm, we want to ensure
      // that we start at (x1, y1) and move towards (x2, y2)
      if(x1 > x2) {
         swap(x1, x2);
         swap(y1, y2);
      }
   }

   const double dx = fabs(x2 - x1);
   const double dy = fabs(y2 - y1);

   double error    = dx * 0.5;
   const int ystep = (y1 < y2) ? 1 : -1;
   int y           = int(y1);

   const int maxX  = int(x2);
   const int xstep = (x1 < x2) ? 1 : -1;

   for(int x = int(x1); x != maxX; x += xstep) {
      if(steep)
         func(y, x);
      else
         func(x, y);

      error -= dy;
      if(error < 0) {
         y += ystep;
         error += dx;
      }
   }
}

void bresenham(Point2 p1, Point2 p2, std::function<void(int, int)> func)
{
   bresenham(p1.x, p1.y, p2.x, p2.y, func);
}

void bresenham(Vector2 p1, Vector2 p2, std::function<void(int, int)> func)
{
   bresenham(p1.x, p1.y, p2.x, p2.y, func);
}

void bresenham(Vector2 p1,
               Vector2 p2,
               AABB bounds,
               std::function<void(int, int)> func)
{
   const auto [a, b] = intersect(bounds, p1, p2);
   bresenham(a, b, func);
}

void bresenham(double x1,
               double y1,
               double x2,
               double y2,
               AABB bounds,
               std::function<void(int, int)> func)
{
   bresenham(Vector2(x1, y1), Vector2(x2, y2), bounds, func);
}

void bresenham(Point2 p1,
               Point2 p2,
               AABBi bounds,
               std::function<void(int, int)> func)
{
   const auto [a, b]
       = intersect(AABB(bounds.left, bounds.top, bounds.right, bounds.bottom),
                   to_vec2(p1),
                   to_vec2(p2));

   // Round to the boundary
   auto correct = [&bounds](const Vector2& u) -> Point2 {
      int x0 = int(std::floor(u.x)), x1 = int(std::ceil(u.x));
      int y0 = int(std::floor(u.y)), y1 = int(std::ceil(u.y));
      int x = (x0 == bounds.left or x0 == bounds.right) ? x0 : x1;
      int y = (y0 == bounds.top or y0 == bounds.bottom) ? y0 : y1;
      return Point2(x, y);
   };

   bresenham(correct(a), correct(b), func);
}

// ---------------------------------------------------------------- plot line AA
//
void plot_line_AA(float x1,
                  float y1,
                  float x2,
                  float y2,
                  std::function<void(int, int, float)> f,
                  bool draw_ends,
                  int width)
{
   using std::swap;

   if(!(std::isfinite(x1) and std::isfinite(y1) and std::isfinite(x2)
        and std::isfinite(y2)))
      return;

   const int half = width / 2;
   auto round     = [&](float x) { return int(x + 0.5f); };

   auto dx = x2 - x1;
   auto dy = y2 - y1;

   bool did_swap = false;
   if(fabsf(dx) < fabsf(dy)) {
      swap(x1, y1);
      swap(x2, y2);
      swap(dx, dy);
      did_swap = true;
   }

   if(x2 < x1) {
      swap(x1, x2);
      swap(y1, y2);
   }

   // We know that x1 < x2, and dy < dx
   auto norm = sqrtf(dx * dx + dy * dy);
   if(norm < 1.0f) return;
   dy /= norm;
   dx /= norm;
   auto c        = -(dy * x1 - dx * y1);
   auto gradient = dy / dx; // -1.0 < gradient < 1.0

   // If we swapped, then we want f(y, x, a). Otherwise f(x, y, a)
   std::function<void(int, int, float)> g;
   if(did_swap)
      g = [&](int x, int y, float a) { f(y, x, a); };
   else
      g = [&](int x, int y, float a) { f(x, y, a); };

   // Suggested weight (before normalizing)
   const auto max_weight = 2.0f;
   auto w                = [&](int x, int y) {
      auto d = fabsf(float(x) * dy - float(y) * dx + c);
      return (max_weight - clamp(d * d, 0.0f, max_weight)) / max_weight;
   };

   //
   auto h = [&](float fx, float fy, float weight) {
      int x = round(fx);
      int y = round(fy);
      for(int i = 0; i < width; ++i) {
         float wgt = w(x, y + i - half);
         g(x, y + i - half, wgt * weight);
      }
   };

   // Find the start-point
   const auto x_start = ceilf(x1);
   const auto x_end   = floorf(x2);
   auto y             = y1 + (x_start - x1) * gradient;

   // The start node
   if(draw_ends) h(x1, y1, x_start - x1);

   // The "middle" of the line
   for(float x = x_start - 1.0f; x <= x_end; x += 1.0f) {
      h(x, y, 1.0f);
      y += gradient;
   }

   // The end node
   if(draw_ends) h(x2, y2, (x2 - x_end));
}

void plot_line_AA(Vector2 a,
                  Vector2 b,
                  std::function<void(int, int, float)> f,
                  bool draw_ends,
                  int width)
{
   plot_line_AA(
       float(a.x), float(a.y), float(b.x), float(b.y), f, draw_ends, width);
}

void plot_line_AA(Point2 a,
                  Point2 b,
                  std::function<void(int, int, float)> f,
                  bool draw_ends,
                  int width)
{
   plot_line_AA(
       float(a.x), float(a.y), float(b.x), float(b.y), f, draw_ends, width);
}

void plot_line_AA(Vector2 a,
                  Vector2 b,
                  AABB bounds,
                  std::function<void(int, int, float)> f,
                  bool draw_ends,
                  int width)
{
   const auto [u, v] = intersect(bounds, a, b);
   if(u.is_finite() and v.is_finite())
      plot_line_AA(
          float(u.x), float(u.y), float(v.x), float(v.y), f, draw_ends, width);
}

void plot_line_AA(Point2 a,
                  Point2 b,
                  AABBi bounds,
                  std::function<void(int, int, float)> f,
                  bool draw_ends,
                  int width)
{
   const auto [u, v]
       = intersect(AABB(bounds.left, bounds.top, bounds.right, bounds.bottom),
                   to_vec2(a),
                   to_vec2(b));

   // Round to the boundary
   auto correct = [&bounds](const Vector2& u) -> Point2 {
      int x0 = int(std::floor(u.x)), x1 = int(std::ceil(u.x));
      int y0 = int(std::floor(u.y)), y1 = int(std::ceil(u.y));
      int x = (x0 == bounds.left or x0 == bounds.right) ? x0 : x1;
      int y = (y0 == bounds.top or y0 == bounds.bottom) ? y0 : y1;
      return Point2(x, y);
   };

   if(u.is_finite() and v.is_finite())
      plot_line_AA(correct(u), correct(v), f, draw_ends, width);
}

// ------------------------------------------------ Quadric Bezier Anti-Aliasing
//
void plot_quad_bezier_AA(const Vector2& A,
                         const Vector2& B,
                         const Vector2& C,
                         std::function<void(int, int, float)> f)
{
   auto eval_quad_bezier = [&](double t, Vector2& ret) {
      assert(t >= 0.0 && t <= 1.0);
      double c_a = square(1.0 - t); // co-efficient
      double c_b = 2.0 * t * (1.0 - t);
      double c_c = square(t);
      ret.x      = c_a * A.x + c_b * B.x + c_c * C.x;
      ret.y      = c_a * A.y + c_b * B.y + c_c * C.y;
   };

   double dist    = (B - A).norm() + (C - B).norm();
   double n_steps = dist / 5.0;    // about once every 5 pixels
   double step    = 1.0 / n_steps; // this is the step size

   struct PA
   {
      PA() = default;
      PA(int x, int y, float a_)
          : p(x, y)
          , a(a_)
      {}

      bool operator<(const PA& o) const
      {
         if(p.x != o.p.x) return p.x < o.p.x;
         if(p.y != o.p.y) return p.y < o.p.y;
         return a > o.a;
      }

      Point2 p;
      float a{0.0};
   };

   std::deque<PA> pts;
   auto g = [&](int x, int y, float a) { pts.emplace_back(x, y, a); };

   Vector2 v0, v1;
   eval_quad_bezier(0.0, v0);
   for(double t = step; t < 1.0; t += step) {
      eval_quad_bezier(t, v1);
      plot_line_AA(float(v0.x), float(v0.y), float(v1.x), float(v1.y), g, true);
      v0 = v1; // Get ready for next step
   }
   plot_line_AA(float(v0.x), float(v0.y), float(C.x), float(C.y), g, true);

   if(pts.size() == 0) return;

   // We only one to draw each pixel once -- because we
   // may be evaluating a cost function
   sort(pts.begin(), pts.end());

   {
      bool first = true;
      PA last;
      for(auto& p : pts) {
         if(first) {
            first = false;
            last  = p;
         } else if(p.p == last.p) {
            // Duplicate
         } else {
            f(last.p.x, last.p.y, last.a);
            last = p;
         }
      }
      f(last.p.x, last.p.y, last.a);
   }
}

void plot_quad_bezier_AA2(
    const Vector2& A,
    const Vector2& B,
    const Vector2& C,
    std::function<void(int, int, float, const Vector2&)> f)
{
   auto eval_quad_bezier = [&](double t, Vector2& ret) {
      assert(t >= 0.0 && t <= 1.0);
      double c_a = square(1.0 - t); // co-efficient
      double c_b = 2.0 * t * (1.0 - t);
      double c_c = square(t);
      ret.x      = c_a * A.x + c_b * B.x + c_c * C.x;
      ret.y      = c_a * A.y + c_b * B.y + c_c * C.y;
   };

   double dist    = (B - A).norm() + (C - B).norm();
   double n_steps = dist / 5.0;    // about once every 5 pixels
   double step    = 1.0 / n_steps; // this is the step size

   struct PA
   {
      PA() = default;
      PA(int x, int y, float a_, Vector2 v_)
          : p(x, y)
          , a(a_)
          , v(v_)
      {}

      bool operator<(const PA& o) const
      {
         if(p.x != o.p.x) return p.x < o.p.x;
         if(p.y != o.p.y) return p.y < o.p.y;
         return a > o.a;
      }

      Point2 p;
      float a{0.0};
      Vector2 v;
   };

   std::deque<PA> pts;
   Vector2 n;
   auto g = [&](int x, int y, float a) { pts.emplace_back(x, y, a, n); };

   Vector2 v0, v1;
   eval_quad_bezier(0.0, v0);
   for(double t = step; t < 1.0; t += step) {
      eval_quad_bezier(t, v1);
      n = (v0 - v1).normalised().clockwise_90();
      plot_line_AA(float(v0.x), float(v0.y), float(v1.x), float(v1.y), g, true);
      v0 = v1; // Get ready for next step
   }
   plot_line_AA(float(v0.x), float(v0.y), float(C.x), float(C.y), g, true);

   if(pts.size() == 0) return;

   // We only one to draw each pixel once -- because we
   // may be evaluating a cost function
   sort(pts.begin(), pts.end());

   {
      bool first = true;
      PA last;
      for(auto& p : pts) {
         if(first) {
            first = false;
            last  = p;
         } else if(p.p == last.p) {
            // Duplicate
         } else {
            f(last.p.x, last.p.y, last.a, last.v);
            last = p;
         }
      }
      f(last.p.x, last.p.y, last.a, last.v);
   }
}

} // namespace perceive
