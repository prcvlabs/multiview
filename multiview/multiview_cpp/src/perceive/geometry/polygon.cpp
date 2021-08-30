
#include "polygon.hpp"

namespace perceive
{
// ---------------------------------------------------- circle-intersection-area

real circles_intersection_area(const real d, // distance between centres
                               const real r) noexcept // radius of circles
{
   // Area of a 'wedge' of a circle: 0.5 * theata * square(r)
   // Area of a 'triangle' in that wedge: square(r)*sin(theta/2)*cos(theta/2)
   // Area of remainder: 0.5 * square(r) * (theta - sin(theta))
   if(d >= 2.0 * r) return 0.0;
   const auto theta = 2.0 * acos(std::clamp(0.5 * d / r, 0.0, 1.0));
   return square(r) * (theta - sin(theta));
}

real circles_union_area(const real d, const real r) noexcept
{
   const auto iarea = circles_intersection_area(d, r);
   return 2.0 * M_PI * square(r) - iarea;
}

real circles_intersection_on_union(const real d, const real r) noexcept
{
   const auto iarea = circles_intersection_area(d, r);
   const auto uarea = 2.0 * M_PI * square(r) - iarea;
   return iarea / uarea;
}

real circles_intersection_area(const Vector2& u,
                               const Vector2& v,
                               const real r) noexcept
{
   return circles_intersection_area((u - v).norm(), r);
}

real circles_union_area(const Vector2& u,
                        const Vector2& v,
                        const real r) noexcept
{
   const auto iarea = circles_intersection_area(u, v, r);
   return 2.0 * M_PI * square(r) - iarea;
}

real circles_intersection_on_union(const Vector2& u,
                                   const Vector2& v,
                                   const real r) noexcept
{
   const auto iarea = circles_intersection_area(u, v, r);
   const auto uarea = 2.0 * M_PI * square(r) - iarea;
   return iarea / uarea;
}

} // namespace perceive
