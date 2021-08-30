
#pragma once

#include "perceive/geometry/euclidean-transform.hpp"

namespace perceive::calibration::n_way
{
// A camera index and its (current) euclidean transform
using index_et_pair = std::pair<int, EuclideanTransform>;

// The cost std::function between a pair of cameras
using cost_fun_type
    = std::function<real(const index_et_pair& camA, const index_et_pair& camB)>;

// The result object
struct Result
{
   real min_val = dNAN;
   vector<EuclideanTransform> ets;
};

/**
 * @param n_cameras     Number of cameras
 * @param ets           Initial camera positions
 * @param edges         Every camera pair that has a constraint
 * @param cost_fun      Score
 * @param end_condition True when optimization should stop
 * @return Optimal camera positions and score
 */
Result
n_way_cam_optimize(const int n_cameras,
                   const vector<EuclideanTransform>& ets,
                   const vector<std::pair<int, int>>& edges,
                   cost_fun_type cost_fun,
                   std::function<bool(real score, int counter)> end_condition,
                   const bool use_nelder_mead,
                   const bool feedback);

} // namespace perceive::calibration::n_way
