
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/geometry/aabb.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/geometry/sparse-hist-cell.hpp"

namespace perceive
{
struct P2DFalsePositiveScores
{
   // All scores are in the range [0..1]
   float p2d_score                  = 0.0f; // pose-skeleton-detect-score
   float p2d_cloud_in_bounds_height = 0.0f; // Sum(sparse-hist prob(height))
   float p2d_in_bounds_height       = 0.0f; // Sum(in-bounds prob(height))
   float p2d_shape                  = 0.0f; // NOT IMPLEMENTED
   //                                          Requires pose classification
   float max_prob_cloud_hgt = fNAN; //
   Point2 max_prob_cloud_xy = {-1, -1};
   vector<SparseHistCell> cloud_weighted_xy; // what gets drawn
};

P2DFalsePositiveScores
calc_prob_p2d_fp_scores(const LocalizationData::Params& loc_params,
                        const Skeleton2D& p2d,
                        const vector<SparseHistCell>* hist_ptr,
                        const AABB& bounds,
                        const real hist_sz,
                        const DistortedCamera& dcam) noexcept;

float calc_prob_p2d_false_positive(const LocalizationData::Params& loc_params,
                                   const Skeleton2D& p2d,
                                   const vector<SparseHistCell>* hist_ptr,
                                   const AABB& bounds,
                                   const real hist_sz,
                                   const DistortedCamera& dcam,
                                   const Vector4f& weights) noexcept;

float calc_prob_p2d_false_positive(const P2DFalsePositiveScores& scores,
                                   const Vector4f& weights) noexcept;

} // namespace perceive
