
#pragma once

#include "stdinc.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/cost-functions/tracklets/track.hpp"
#include "perceive/cost-functions/tracks/node-sequence.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
struct TrackerResult
{
   int start_frame                        = -1;
   int n_frames                           = -1;
   int hist_w                             = -1;
   int hist_h                             = -1;
   Vector2 top_left                       = {dNAN, dNAN};
   real hist_sz                           = dNAN;
   real frame_duration                    = dNAN;
   vector<tracks::NodeSequence> seqs      = {};
   vector<TrackFalsePositiveInfo> fp_info = {};

   FowlkesResult to_fowlkes_result() const noexcept;
};

struct PostProcessParams final : public MetaCompatible
{
   virtual ~PostProcessParams() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   float prob_fp_median = 0.25f;
   float prob_fp_stddev = 0.05f;

   float dist_median = 0.10f;
   float dist_stddev = 0.05f;

   float duration_median = 3.00f;
   float duration_stddev = 1.00f;

   float score_threshold = 0.25f;
};

TrackerResult post_process(const TrackerResult& tr,
                           const PostProcessParams& params) noexcept;

} // namespace perceive
