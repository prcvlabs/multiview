
#pragma once

#include "pipeline-output.hpp"

#include "perceive/cost-functions/tracklets/track.hpp"
#include "perceive/cost-functions/tracks/labeled-track-point.hpp"

namespace perceive::pipeline
{
struct TestOutput
{
   bool test_passes = true;

   unsigned start_frame   = 0;
   unsigned calc_n_frames = 0;

   /// Tracks may be at different frame rates
   vector<Track> ground_truth;
   vector<Track> calculated;

   real gt_frame_rate   = 0.0;
   real calc_frame_rate = 0.0;

   AABB hist_bounds;
   real hist_sz = 0.0;

   /// Given a track-id, will find the index in 'grouth-truth' or 'calculated'
   std::unordered_map<unsigned, unsigned> gt_lookup;
   std::unordered_map<unsigned, unsigned> calc_lookup;

   unsigned total_true_positives  = 0;
   unsigned total_false_positives = 0;
   unsigned total_false_negatives = 0;

   size_t total_identity_switches = 0;
   size_t total_track_frags       = 0;
   real total_tp_distances        = 0.0; // true-positive euclidean distances
   real total_tp_iou = 0.0;          // true-positive interseciton over union
   size_t total_mt   = 0;            // total number of "mostly tracked" tracks
   size_t total_ml   = 0;            // total number of "mostly lost" tracks
   size_t total_pt() const noexcept; // total "partially tracked" tracks

   SampleStatistics theta_stats = {};
   unsigned total_theta_errors  = 0;

   real proportion_with_gaze = 0.0;

   real f1() const noexcept;

   // One element for each frame
   struct FrameInfo
   {
      struct GtMatchInfo
      {
         LabeledTrackPoint ltp;    // The ground-truth track-point
         int calc_index      = -1; // within a given frame
         bool is_theta_error = false;
         bool is_track_break = false; // calc-track-id has changed

         // NAN when no match/missing data
         float theta_delta(const FrameInfo&) const noexcept;
      };

      int gt_t   = -1; // the 'gt' frame-number
      int calc_t = -1; // 'calc' frame-number

      vector<GtMatchInfo> ltp_gt; //   .size() == ltp_gt.size()
      vector<LabeledTrackPoint> ltp_calc;

      vector<int> calc_matches; // .size() == ltp_calc.size(), -1 is no match

      string to_string() const noexcept;
   };

   vector<FrameInfo> frames;

   void init(const unsigned start_frame,
             const unsigned calc_n_frames, // calc-n-frames
             const vector<Track>& ground_truth,
             const real gt_frame_duration,
             const vector<Track>& calculated,
             const real calc_frame_duration,
             const AABB& hist_bounds,
             const real hist_sz,
             const real track_radius,
             const real gaze_theta_threshold) noexcept(false);

   string to_string() const noexcept;
   friend string str(const TestOutput&) noexcept;
};

TestOutput compare_tracker_output(const FowlkesResult& ground_truth,
                                  const FowlkesResult& calculated,
                                  const real radius) noexcept(false);

TestOutput
compare_tracker_output(const unsigned start_frame,
                       const unsigned calc_n_frames, // calc-n-frames
                       const vector<Track>& ground_truth,
                       const real gt_frame_duration,
                       const vector<Track>& calculated,
                       const real calc_frame_duration,
                       const AABB& hist_bounds,
                       const real hist_sz,
                       const real track_radius,
                       const real gaze_theta_threshold) noexcept(false);

} // namespace perceive::pipeline
