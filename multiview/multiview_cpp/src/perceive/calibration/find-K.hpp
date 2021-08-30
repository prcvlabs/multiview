
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"

namespace perceive::calibration
{
class FindKCostFunctor
{
 public:
   Lp_norm_t method{Lp_norm_t::L1}; // Method used in homography error
   bool single_f{false}; // Constrain fx == fy; lowers n-params from 4 to 3
   unsigned n_params() const { return single_f ? 3 : 4; }

   // The set of world points, and corner (i.e., image) points
   std::vector<Vector2r> W;
   std::vector<real> weights; // Applied in cost function |Cs|==|weights|
   std::vector<std::vector<Vector2r>> Cs; // Corner sets
   std::vector<Matrix3r> Hs;              // The pre-fitted homographies

   // Evaluate the cost function for the passed set of parameters
   // X is a 3x3 homography (H) in row major order
   // the result is the mean-sq-err for (C[i] - H W[i]) in R2
   real evaluate(const real* X) const;

   // Human readable print-out shows W and C
   std::string to_string() const;
};

Matrix3r estimate_K_Zhang(const vector<Matrix3r>& Hs, bool feedback);

Matrix3r estimate_K(const vector<Vector3r>& W,
                    const vector<vector<Vector3r>>& Cs,
                    const vector<Matrix3r>& Hs, bool feedback);

Matrix3r estimate_K_Zhang(const unsigned nx, const unsigned ny,
                          const vector<vector<Vector3r>>& Cs,
                          const bool feedback);

Matrix3r estimate_K(const unsigned nx, const unsigned ny,
                    const vector<vector<Vector3r>>& Cs, const bool feedback);

real refine_K(Matrix3r& K, const vector<Vector3r>& W,
              const vector<vector<Vector3r>>& Cs, const vector<Matrix3r>& Hs,
              bool feedback);

} // namespace perceive::calibration
