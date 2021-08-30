
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"

namespace perceive::calibration
{
///
/// Class for efficiently evaluating a 'find-homography' cost function
///

// What not use a closure, like good c++11?? Because this object
// is meant to work with python bindings!
class FindHomographyCostFunctor
{
 public:
   // Lp norm is appled to the homography errors as a vector
   Lp_norm_t method{Lp_norm_t::L1};

   // The set of world points, and corner (i.e., image) points
   std::vector<Vector2r> W, C;

   // Evaluate the cost function for the passed set of parameters
   // X is a 3x3 homography (H) in row major order
   // the result is the mean-sq-err for (C[i] - H W[i]) in R2
   real evaluate(const real X[9]) const;

   static real homography_err(const Matrix3r& H, const std::vector<Vector2r>& W,
                              const std::vector<Vector2r>& C, Lp_norm_t method);

   // Unpacks X[9] (ie, the paramaters) as a normalized homography
   Matrix3r unpack(const real X[9]) const;

   // Human readable print-out shows W and C
   std::string to_string() const;
};

// Such that C[i] = H * W[i]
real estimate_homography_LLS(const std::vector<Vector3r>& W,
                             const std::vector<Vector3r>& C, Matrix3r& H);

real estimate_homography_LLS(const std::vector<Vector2r>& W,
                             const std::vector<Vector2r>& C, Matrix3r& H);

inline void translate_homographies(vector<Matrix3r>& Hs, const Vector2 C)
{
   Matrix3r HC = Matrix3r::Identity();
   Matrix3r t;
   HC(0, 2) = C(0);
   HC(1, 2) = C(1);
   for(auto& H : Hs) {
      t = HC * H;
      H = t;
   }
}

void H_to_r1r2t(const Matrix3r& H, Vector3& ssa, Vector3r& t,
                bool feedback = false);
void r1r2t_to_H(const Vector3& ssa, const Vector3r& t, Matrix3r& H);
void H_to_r1r2t(const Matrix3r& H, Vector6r& X);
void r1r2t_to_H(const Vector6r& X, Matrix3r& H);

// Given 8 values, calculates the last value of the homography
// (h33), such that the homography has determinant 1.0.
// The value of H(2, 2) is ignored and over-written.
void complete_homography(Matrix3r& H);

} // namespace perceive::calibration
