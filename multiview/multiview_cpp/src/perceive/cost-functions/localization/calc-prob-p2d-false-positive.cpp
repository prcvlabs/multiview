
#include "calc-prob-p2d-false-positive.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/geometry/human-heights.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"

namespace perceive
{
// --------------------------------------------- calc-p2d-cloud-in-bounds-height
//
static float
calc_p2d_cloud_in_bounds_height(const LocalizationData::Params& loc_params,
                                const Skeleton2D& p2d,
                                const vector<SparseHistCell>* hist_ptr,
                                const AABB& bounds,
                                const real hist_sz,
                                const DistortedCamera& dcam,
                                P2DFalsePositiveScores& ret) noexcept
{
   constexpr float k_no_hist_score = 0.150f; // If there's no point cloud data
   constexpr float k_min_score     = 0.001f; // Scores never less than this

   if(hist_ptr == nullptr) return k_no_hist_score;

   const auto top_left    = bounds.top_left();
   auto unproject_hist_xy = [&](const Point2& xy) {
      return FloorHistogram::unproject_hist_xyz(xy, top_left, hist_sz);
   };

   auto sum      = 0.0f;
   auto wgt_sum  = 0.0f;
   auto best_val = std::numeric_limits<float>::lowest();

   ret.cloud_weighted_xy = *hist_ptr;

   for(auto& cell : ret.cloud_weighted_xy) {
      const float wgt = cell.count;
      const auto X    = unproject_hist_xy(cell.xy);
      if(!bounds.contains(X.x, X.y)) continue;
      const auto hcp  = hist_cell_plausibility(loc_params, p2d, dcam, X);
      const auto prob = hcp.probability();
      if(false) {
         INFO(format(
             "hcp-height, xy = {}, X = {}, hgt = {}, prob = {}, wgt = {}",
             str(cell.xy),
             str(X),
             hcp.height,
             prob,
             wgt));
      }
      Expects(prob >= 0.0f);
      Expects(prob <= 1.0f);
      const auto weighted_prob = wgt * prob;
      if(weighted_prob > best_val) {
         ret.max_prob_cloud_hgt = hcp.height;
         ret.max_prob_cloud_xy  = cell.xy;
         best_val               = weighted_prob;
      }

      cell.count = weighted_prob;

      wgt_sum += wgt;
      sum += weighted_prob;
   }

   const auto avg = sum / wgt_sum;
   return (!std::isfinite(avg) || avg < k_min_score) ? k_min_score : avg;
}

// --------------------------------------------------- calc-p2d-in-bounds-height
//
static float calc_p2d_in_bounds_height(const Skeleton2D& p2d,
                                       const AABB& bounds,
                                       const real hist_sz,
                                       const DistortedCamera& dcam,
                                       P2DFalsePositiveScores& ret) noexcept
{
   return 0.0f;
}

// ----------------------------------------------------- calc-prob-p2d-fp-scores
//
P2DFalsePositiveScores
calc_prob_p2d_fp_scores(const LocalizationData::Params& loc_params,
                        const Skeleton2D& p2d,
                        const vector<SparseHistCell>* hist_ptr,
                        const AABB& bounds,
                        const real hist_sz,
                        const DistortedCamera& dcam) noexcept
{
   P2DFalsePositiveScores ret;

   // -- (*) -- p2d-score
   ret.p2d_score = std::clamp<float>(1.0f - float(p2d.score()), 0.0f, 1.0f);

   // -- (*) -- (point) cloud-in-bounds-height
   ret.p2d_cloud_in_bounds_height = std::clamp<float>(
       1.0f
           - calc_p2d_cloud_in_bounds_height(
               loc_params, p2d, hist_ptr, bounds, hist_sz, dcam, ret),
       0.0f,
       1.0f);

   // -- (*) -- in-bounds-height
   ret.p2d_in_bounds_height = std::clamp<float>(
       1.0f - calc_p2d_in_bounds_height(p2d, bounds, hist_sz, dcam, ret),
       0.0f,
       1.0f);

   // -- (*) -- shape score, from skeleton3d
   ret.p2d_shape = std::clamp<float>(
       1.0f - shape_score(p2d.best_3d_result()), 0.0f, 1.0f);

   return ret;
}

// -----------------------------------------------------------------------------
//
float calc_prob_p2d_false_positive(const P2DFalsePositiveScores& x,
                                   const Vector4f& weights) noexcept
{
   const auto prob_fp
       = x.p2d_score * weights[0] + x.p2d_cloud_in_bounds_height * weights[1]
         + x.p2d_in_bounds_height * weights[2] + x.p2d_shape * weights[3];
   return 1.0f
          - logish_prob(
              logish_prob(std::clamp<float>(1.0f - prob_fp, 0.0f, 1.0f)));
}

// -----------------------------------------------------------------------------
//
float calc_prob_p2d_false_positive(const LocalizationData::Params& loc_params,
                                   const Skeleton2D& p2d,
                                   const vector<SparseHistCell>* hist_ptr,
                                   const AABB& bounds,
                                   const real hist_sz,
                                   const DistortedCamera& dcam,
                                   const Vector4f& weights) noexcept
{
   const auto x = calc_prob_p2d_fp_scores(
       loc_params, p2d, hist_ptr, bounds, hist_sz, dcam);
   return calc_prob_p2d_false_positive(x, weights);
}

} // namespace perceive
