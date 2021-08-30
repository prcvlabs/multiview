
#include "triangulation.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "stdinc.hpp"

namespace perceive
{
// ----------------------------------------------------------------- Triangulate

Vector3 triangulate(const Vector2& x0, const Vector2& x1, const Matrix3r& R,
                    const Vector3& t) noexcept
{
   const bool feedback = false;

   MatrixXr A = MatrixXr::Zero(4, 4);
   A(0, 0)    = -1.0;
   A(0, 2)    = x0.x;
   A(1, 1)    = -1.0;
   A(1, 2)    = x0.y;

   for(auto c = 0; c < 3; ++c) {
      A(2, c) = x1.x * R(2, c) - R(0, c);
      A(3, c) = x1.y * R(2, c) - R(1, c);
   }

   A(2, 3) = x1.x * t(2) - t(0);
   A(3, 3) = x1.y * t(2) - t(1);

   if(feedback) {
      MatrixXr U, V;
      VectorXr D;
      svd_UDV(A, U, D, V);
      INFO(format("triangulate"));
      MatrixXr DD = MatrixXr::Zero(4, 4);
      for(auto i = 0; i < 4; ++i) DD(i, i) = D(i);
      cout << "A = " << endl << A << endl;
      cout << "U = " << endl << U << endl;
      cout << "D = " << endl << DD << endl;
      cout << "V = " << endl << V << endl;
      cout << format("  condition-number {} (last two: {})", D(0) / D(3),
                     D(2) / D(3))
           << endl;
   }

   VectorXr p3;
   svd_thin(A, p3);

   Vector3 ret(p3(0), p3(1), p3(2));
   ret /= p3(3);
   return ret;
}

// -------------------------------------------------------------- Intersect Rays

Vector3 intersect_rays(const Vector3& u0, const Vector3& u1, const Vector3& v0,
                       const Vector3& v1)
{
#define SMALL_NUM 1e-9
   Vector3 u = u1 - u0;
   Vector3 v = v1 - v0;
   Vector3 w = u0 - v0;

   double a = u.dot(u); // always >= 0
   double b = u.dot(v);
   double c = v.dot(v); // always >= 0
   double d = u.dot(w);
   double e = v.dot(w);
   double D = a * c - b * b; // always >= 0
   double sc, tc;

   // compute the line parameters of the two closest points
   if(D < SMALL_NUM) { // the lines are almost parallel
      sc = 0.0;
      tc = (b > c ? d / b : e / c); // use the largest denominator
   } else {
      sc = (b * e - c * d) / D;
      tc = (a * e - b * d) / D;
   }

   Vector3 ret = 0.5 * (u0 + (sc * u) + v0 + (tc * v));
   return ret;
#undef SMALL_NUM
}

Vector3 intersect_rays_2(const Vector3& C0, const Vector3& u1,
                         const Vector3& C1, const Vector3& v1)
{
   // Assume A0 and B0 are on the epipolar plane,
   // then 'C' is the "average" point to complete the plane
   Plane pi(C0, C1, 0.5 * (u1 + v1));
   assert(pi.is_finite());

   // Image u1 and u1 onto pi
   return intersect_rays(C0, pi.image(u1), C1, pi.image(v1));
}

real intersect_rays_2_plane_err(const Vector3& C0, const Vector3& u1,
                                const Vector3& C1, const Vector3& v1)
{
   Plane pi(C0, C1, 0.5 * (u1 + v1));
   assert(pi.is_finite());
   return 0.5 * (fabs(pi.side(u1)) + fabs(pi.side(v1)));
}

} // namespace perceive
