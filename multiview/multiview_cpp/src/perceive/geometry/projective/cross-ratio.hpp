
#pragma once

#include "perceive/geometry/vector.hpp"

namespace perceive
{
inline real cross_ratio(real a, real b, real c, real d)
{
   return ((a - c) * (b - d)) / ((b - c) * (a - d));
}

inline real cross_ratio(const Vector2& A, const Vector2& B, const Vector2& C,
                        const Vector2& D)
{
   Vector2 n = (A - D);
   real w    = -n.dot(A);
   real u    = n.x;
   real v    = n.y;

   // Find the point-line distances
   real a = u * A[0] + v * A[1] + w; //   # i.e., 0
   real b = u * B[0] + v * B[1] + w;
   real c = u * C[0] + v * C[1] + w;
   real d = u * D[0] + v * D[1] + w;

   return cross_ratio(a, b, c, d);
}

inline real cross_ratio(const Vector3r& A, const Vector3r& B, const Vector3r& C,
                        const Vector3r& D)
{
   return cross_ratio(Vector2(A(0), A(1)) / A(2), Vector2(B(0), B(1)) / B(2),
                      Vector2(C(0), C(1)) / C(2), Vector2(D(0), D(1)) / D(2));
}

} // namespace perceive
