
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"

namespace perceive::calibration
{
class FindFCostFunctor
{
 private:
   Matrix3r K1_, K2_;          // Intrinsic parameters for cameras 1 and 2
   Matrix3r K1_inv_, K2t_inv_; // for calculating the Fundamental matrix

 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   std::vector<Vector3r> pts1, pts2; // Corresponding pairs of points

   const Matrix3r& K1() const { return K1_; }
   const Matrix3r& K2() const { return K2_; }

   void set_K1(const Matrix3r& K);
   void set_K2(const Matrix3r& K);

   // Evaluate the cost function for the passed set of 5 parameters
   // The first three are a rotation in spherical-axis-angle format
   // The last two are the inclincation and azimuth of "t"
   real evaluate(const real* X) const;

   // Unpack "X" into a rotation matrix and "t"
   void unpack(const real* X, Matrix3r& R, Vector3& t) const;

   unsigned n_params() const { return 5; }

   // Human readable print-out shows W and C
   std::string to_string() const;

   //
   real estimate_baseline(unsigned nx,
                          unsigned ny,
                          real square_size,
                          const Matrix3r& R,
                          const Vector3& t) const;
};

// Runs the 8-point algorithm to find a linear-least-square estimate of F
// pts2^t F pts1
Matrix3r longuet_higgins(const std::vector<Vector3r>& pts1,
                         const std::vector<Vector3r>& pts2);

// pts2^t F pts1
Matrix3r estimate_F(const std::vector<Vector3r>& pts1,
                    const std::vector<Vector3r>& pts2);

// pts2^t F pts1
Matrix3r estimate_E(const std::vector<Vector3r>& n_pts1, // normalized points
                    const std::vector<Vector3r>& n_pts2);

Matrix3r condition_F(const Matrix3r& F);
Matrix3r condition_E(const Matrix3r& E);

// pts2^t F pts1
void estimate_Rt_from_E(const Matrix3r& E0,
                        const std::vector<Vector3r>& pts1, // normalized points
                        const std::vector<Vector3r>& pts2,
                        Quaternion& q, // Out
                        Vector3& t,
                        const bool optimize = true,
                        const bool feedback = false); // out

// pts2^t F pts1
void estimate_Rt(const std::vector<Vector3r>& n_pts1, // normalized points
                 const std::vector<Vector3r>& n_pts2,
                 Quaternion& q, // Out
                 Vector3& t,
                 bool feedback = false);

Matrix3r make_essential_matrix(const Quaternion& q, const Vector3& t);

// pts2^t F pts1
real xFx(const Matrix3r& F,
         const std::vector<Vector3r>& pts1,
         const std::vector<Vector3r>& pts2);

// l = F pts1, normalize l, err += fabs(x.dot(l))
real xFl(const Matrix3r& F,
         const std::vector<Vector3r>& pts1,
         const std::vector<Vector3r>& pts2);

real xFl_lFx(const Matrix3r& F,
             const std::vector<Vector3r>& pts1,
             const std::vector<Vector3r>& pts2);

// p2^t F p1
real xFl_lFx(const Matrix3r& F, const Vector2& p1, const Vector2& p2);
real xFl_lFx(const Matrix3r& F, const Vector3r& x1, const Vector3r& x2);

} // namespace perceive::calibration
