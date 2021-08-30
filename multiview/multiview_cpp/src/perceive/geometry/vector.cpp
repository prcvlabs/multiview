
#include "perceive/geometry/rotation.hpp"
#include "vector.hpp"

namespace perceive
{
// ----------------------------------------------------------- Quantize Funciton
std::array<std::pair<Point2, float>, 4> quantize(const Vector2& X) noexcept
{
   const auto raw_dx = X.x - std::round(X.x);
   const auto raw_dy = X.y - std::round(X.y);
   const auto x0     = int(std::round(X.x));
   const auto y0     = int(std::round(X.y));
   const auto u      = (raw_dx < 0.0) ? -1 : 1;
   const auto v      = (raw_dy < 0.0) ? -1 : 1;
   const auto dx     = 1.0 - std::fabs(raw_dx);
   const auto dy     = 1.0 - std::fabs(raw_dy);

   std::array<std::pair<Point2, float>, 4> out;
   out[0].first = Point2(x0 + 0, y0 + 0);
   out[1].first = Point2(x0 + u, y0 + 0);
   out[2].first = Point2(x0 + u, y0 + v);
   out[3].first = Point2(x0 + 0, y0 + v);

   out[0].second = float(dx * dy);
   out[1].second = float((1.0 - dx) * dy);
   out[2].second = float((1.0 - dx) * (1.0 - dy));
   out[3].second = float(dx * (1.0 - dy));

   return out;
}

} // namespace perceive
