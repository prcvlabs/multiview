
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
class Tracks;

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

// The keep/drop rule of `post_process` for one sequence of `tr`
TrackFalsePositiveInfo calc_fp_info(const TrackerResult& tr,
                                    const PostProcessParams& params,
                                    const tracks::NodeSequence* ptr);

TrackerResult post_process(const TrackerResult& tr,
                           const PostProcessParams& params) noexcept;

// Concatenates the sequences of consecutive tracking windows by id, as the
// `tracks` task does at the end of a run. Empty if `windows` is empty.
TrackerResult make_tracker_result(const vector<const Tracks*>& windows,
                                  const int start_frame,
                                  const int n_frames,
                                  const Vector2& top_left,
                                  const real hist_sz,
                                  const real frame_duration) noexcept;

// `post_process`, then `smooth_tracks`: the end of a batch run. The result
// before smoothing is copied to `presmooth` if it is not null.
FowlkesResult finalise_tracks(const TrackerResult& tr,
                              const PostProcessParams& post_process_params,
                              const fowlkes::Params& fowlkes_params,
                              FowlkesResult* presmooth = nullptr) noexcept;

} // namespace perceive
