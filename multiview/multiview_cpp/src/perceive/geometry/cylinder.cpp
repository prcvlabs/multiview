
#include "cylinder.hpp"

#include "fitting-planes.hpp"
#include "perceive/utils/eigen-helpers.hpp"

namespace perceive
{
void ProjectiveFloorCylinder::init(const Plane& left,
                                   const Plane& right,
                                   const Plane& top)
{
   top_   = top.normalised_plane();
   left_  = left.normalised_plane();
   right_ = right.normalised_plane();

   { // The center point should be the same as the camera center
      C_ = intersection_of_3_planes(top, left, right);
   }

   { // The bisecting plane...
      const auto dot = left_.xyz().dot(right_.xyz());
      const auto L   = (dot < 0.0) ? -left_.xyz() : left_.xyz();
      const auto R   = right_.xyz();
      Expects(L.dot(R) >= 0.0);
      const auto N = (L + R).normalised();
      C_p3_        = Plane(N, -N.dot(C_));
   }

   is_init_ = true;
}

void ProjectiveFloorCylinder::init(const Plane& top,
                                   const Plane& C_p3,
                                   const Vector3& C)
{
   top_  = top.normalised_plane();
   C_    = C;
   C_p3_ = C_p3.normalised_plane();
   Expects(std::fabs(top_.side(C)) < 1e-3);
   Expects(std::fabs(C_p3_.side(C)) < 1e-3);
   is_init_ = true;
}

Cylinder ProjectiveFloorCylinder::realize(const real height) const noexcept
{
   if(!top_.is_finite() || !C_p3_.is_finite()) return {};

   auto calc_X = [&]() {
      MatrixXr P(2, 3);
      P(0, 0)    = top_(0);
      P(0, 1)    = top_(1);
      P(0, 2)    = top_(2) * height + top_(3);
      P(1, 0)    = C_p3_(0);
      P(1, 1)    = C_p3_(1);
      P(1, 2)    = C_p3_(2) * height + C_p3_(3);
      MatrixXr N = matrix_kernel(P);
      return Vector3(N(0) / N(2), N(1) / N(2), 0.0);
   };

   Cylinder Cy;
   Cy.X      = calc_X(); // ON C_p3, and z = 0.0, and C_p3-top intersction-z
   Cy.radius = 0.2;      // TODO
   Cy.height = height;

   Expects(std::fabs(C_p3_.side(Cy.X)) < 1e-3);
   Expects(std::fabs(top_.side(Cy.X + Vector3(0.0, 0.0, height))) < 1e-3);

   return Cy;
}

}; // namespace perceive
