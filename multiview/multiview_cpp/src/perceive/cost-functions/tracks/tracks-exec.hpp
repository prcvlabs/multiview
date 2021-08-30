
#pragma once

#include "node-sequence.hpp"

#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
struct LocalizationData;
}

namespace perceive
{
class Tracks
{
 public:
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      // Do not touch nodes in Seqeuneces in frame [0..n) of `prev-tracklet`
      unsigned fixed_window_prev_tracklet = 6;
      uint32_t random_seed = 0; // pass 0 for a randomly generated seed
      Vector4f false_positive_weight_vector = {0.25f,  // p2d-score
                                               0.25f,  // cloud-in-bounds height
                                               0.00,   // in-bounds height
                                               0.25f}; // p2d-shape

      // TODO, we need to replace this threshold with a global cost track
      float hungarian_score_threshold = 0.10f;

      float edge_d0_delta = -0.80f;
      float edge_d0_scale = 2.0f; // [-1..1]
      float edge_d1_delta = -0.5f;
      float edge_d1_scale = 2.0f; // [-1..1]
      float edge_dt_delta = -0.1f;
      float edge_dt_scale = 2.0f; // [-1..1]

      float merge_threshold = 0.0f;

      string export_training_data_prefix = ""s;

      FowlkesResult ground_truth = {}; // required for export-training-data
      string pose_classifier     = ""s;

      enum Method : int { HUN1 = 0, HUN2, SP1 };
      Method method = SP1;

      static const char* str(const Method) noexcept;
      static Method to_method(const string_view) noexcept;
   };

   CUSTOM_NEW_DELETE(Tracks)

   vector<tracks::NodeSequence> seqs;

   // Only populated if `export_training_data` is set
   vector<calibration::TrainingDataPoint> training_data;

   int max_id = -1;  // The max id found so far (including all previous tracks)
   int w = 0, h = 0; // width and height
   int start_frame          = -1;
   int n_frames             = 0; // total number of frames
   int max_frames_per_track = 0;
   real hist_sz             = dNAN;
   AABB aabb                = AABB::nan();

   size_t memory_usage() const noexcept;

   string to_string() const noexcept;

   friend string str(const Tracks& tracks) noexcept
   {
      return tracks.to_string();
   }
};

// ------------------------------------------------------------------ calc track

Tracks calc_tracks(const SceneDescription& scene_desc,
                   const LocalizationData::Params& loc_params,
                   const Tracklet::Params& tracklet_params,
                   const Tracks::Params& params,
                   const real frame_duration,
                   const int max_frames_per_tracklet,
                   const Tracklet* tracklet,
                   const Tracklet* prev_tracklet,
                   Tracks* prev_track,
                   std::function<bool()> is_cancelled,
                   const string_view outdir,
                   const bool feedback) noexcept(false);

} // namespace perceive
