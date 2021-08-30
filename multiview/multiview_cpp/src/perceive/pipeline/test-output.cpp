
#include "test-output.hpp"

#include "perceive/geometry/circle.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"
#include "perceive/pipeline/pipeline-input.hpp"

#define This TestOutput

namespace perceive::pipeline
{
// -------------------------------------------------------- FrameInfo::to-string

string This::FrameInfo::to_string() const noexcept
{
   auto gt_ss = [&]() {
      std::stringstream ss{""};
      for(size_t i = 0; i < ltp_gt.size(); ++i) {
         const auto& ltp = ltp_gt[i].ltp;
         ss << format(
             "      #{} [{:5.3f}, {:5.3f}, {}] => calc={}, is-break={}\n",
             ltp.track_id,
             ltp.tp.x,
             ltp.tp.y,
             ltp.tp.t,
             ltp_gt[i].calc_index,
             str(ltp_gt[i].is_track_break));
      }
      return ss.str();
   };

   auto ltp_calc_ss = [&]() {
      std::stringstream ss{""};
      const size_t n_calc = ltp_calc.size();
      for(size_t i = 0; i < n_calc; ++i) {
         ss << format("      #{} [{:5.3f}, {:5.3f}, {}] => {}\n",
                      ltp_calc[i].track_id,
                      ltp_calc[i].tp.x,
                      ltp_calc[i].tp.y,
                      ltp_calc[i].tp.t,
                      (i >= calc_matches.size() ? -1 : calc_matches[i]));
      }
      return ss.str();
   };

   return format(R"V0G0N(
FrameInfo: gt-t={} / calc-t={},
   Ground-truth:
{}
   Calc (i.e., tracker):
{}
)V0G0N",
                 gt_t,
                 calc_t,
                 gt_ss(),
                 ltp_calc_ss());
}

// ------------------------------------------------------------------------ init
//
void This::init(const unsigned start_frame_,
                const unsigned calc_n_frames_, // calc-n-frames
                const vector<Track>& ground_truth_,
                const real gt_frame_duration_,
                const vector<Track>& output_data_,
                const real calc_frame_duration_,
                const AABB& hist_bounds_,
                const real hist_sz_,
                const real track_radius,
                const real gaze_theta_threshold) noexcept(false)
{
   if(std::fabs(gt_frame_duration_ - calc_frame_duration_) > 1e-3
      && start_frame_ != 0)
      FATAL(format("Start-frame must be 0 when testing different"
                   " frame-rate durations. Sorry!"));

   { // Set up basics
      this->gt_frame_rate   = 1.0 / gt_frame_duration_;
      this->calc_frame_rate = 1.0 / calc_frame_duration_;
      this->start_frame     = start_frame_;
      this->calc_n_frames   = calc_n_frames_;
      this->hist_bounds     = hist_bounds_;
      this->hist_sz         = hist_sz_;

      Expects(std::isfinite(this->gt_frame_rate));
      Expects(std::isfinite(this->calc_frame_rate));
   }

   { // Set up tracks
      ground_truth = ground_truth_;
      calculated   = output_data_;
   }

   // For dealing with different frame rates
   const auto frame_pairs
       = calc_frame_pairs(calc_frame_rate, gt_frame_rate, int(calc_n_frames));
   // for(auto xy : frame_pairs) { INFO(format("{}  =>  {}", xy.x, xy.y)); }

   { // Set up 'lookups', track.track_id ==> index in `ground_truth` of `calc`
      auto make_lookup = [](const vector<Track>& tracks) {
         std::unordered_map<unsigned, unsigned> lookup;
         lookup.reserve(tracks.size());
         for(auto i = 0u; i < tracks.size(); ++i) {
            const unsigned track_id = unsigned(tracks[i].id);
            if(lookup.count(track_id) > 0)
               throw std::runtime_error(
                   format("duplicate track-id found: {}", track_id));
            lookup.insert({track_id, i});
         }
         return lookup;
      };

      gt_lookup   = make_lookup(ground_truth);
      calc_lookup = make_lookup(calculated);
   }

   { // create labelled trackpoints
      frames.resize(frame_pairs.size());
      for(auto i = 0u; i < frame_pairs.size(); ++i) {
         auto& f    = frames[i];
         f.calc_t   = int(start_frame) + frame_pairs[i].x;
         f.gt_t     = int(start_frame) + frame_pairs[i].y;
         f.ltp_calc = extract_labeled_trackpoints(calculated, f.calc_t);
         f.calc_matches.resize(f.ltp_calc.size());
         std::fill(begin(f.calc_matches), end(f.calc_matches), -1);

         // Set up the GtMatchInfo object
         const auto ltps = extract_labeled_trackpoints(ground_truth, f.gt_t);
         f.ltp_gt.resize(ltps.size());
         for(auto i = 0u; i < ltps.size(); ++i) f.ltp_gt[i].ltp = ltps[i];

         // Some sanity checks
         for(const auto& gt : f.ltp_gt) Expects(gt.ltp.tp.t == f.gt_t);
         for(const auto& ltp : f.ltp_calc) Expects(ltp.tp.t == f.calc_t);
      }
   }

   { // Assocate each calculated label with (at most 1) gt label
      auto associate_ltps = [&](FrameInfo& frame) {
         auto& gt           = frame.ltp_gt;
         const auto& calc   = frame.ltp_calc;
         auto& calc_matches = frame.calc_matches;
         Expects(calc_matches.size() == calc.size());
         std::fill(begin(calc_matches), end(calc_matches), -1);

         const auto raw_pp = hungarian_algorithm(
             unsigned(gt.size()),
             unsigned(calc.size()),
             [&](unsigned gt_idx, unsigned calc_idx) {
                const auto& ltp0 = gt[gt_idx].ltp;
                const auto& ltp1 = calc[calc_idx];
                Expects(std::find(cbegin(frame_pairs),
                                  cend(frame_pairs),
                                  Point2(ltp1.tp.t - int(start_frame),
                                         ltp0.tp.t - int(start_frame)))
                        != cend(frame_pairs));
                return double((ltp0.tp.xy() - ltp1.tp.xy()).quadrance());
             });

         for(const auto& pp : raw_pp) {
            const auto& ltp0 = gt[pp.first].ltp;
            const auto& ltp1 = calc[pp.second];

            // is the match good-enough
            const real cell_dist = real((ltp0.tp.xy() - ltp1.tp.xy()).norm());
            const real dist      = cell_dist * hist_sz;

            if(dist < 2.0 * track_radius) { // we're good
               gt[pp.first].calc_index = int(pp.second);
               calc_matches[pp.second] = int(pp.first);
               total_true_positives += 1;
            }
         }

         for(const auto o : calc_matches)
            if(o == -1) total_false_positives += 1;

         for(const auto& o : frame.ltp_gt)
            if(o.calc_index == -1) total_false_negatives += 1;
      };

      for(unsigned i = 0; i < calc_n_frames; ++i) associate_ltps(frames[i]);
   }

   { // Calculate theta errors
      auto theta_delta = [&](const auto& frame, const auto& gt) {
         if(!std::isfinite(gt.ltp.tp.gaze_direction)) return NAN;
         if(gt.calc_index == -1) return NAN;
         const auto theta0 = gt.ltp.tp.gaze_direction;
         const auto theta1
             = frame.ltp_calc[size_t(gt.calc_index)].tp.gaze_direction;
         return angle_diff(theta0, theta1);
      };

      vector<real> theta_deltas;
      total_theta_errors = 0;
      for(auto& frame : frames) {
         for(auto& gt : frame.ltp_gt) {
            const auto delta = real(theta_delta(frame, gt));
            gt.is_theta_error
                = std::isfinite(delta) && delta > gaze_theta_threshold;
            if(std::isfinite(delta)) { theta_deltas.push_back(delta); }
            if(gt.is_theta_error) total_theta_errors += 1;
         }
      }

      if(theta_deltas.size() > 0)
         theta_stats
             = calc_sample_statistics(begin(theta_deltas), end(theta_deltas));
   }

   { // calculate id-switches
      // Count how many calc-tracks have more than 1 id.
      // Okay, now you have the idea, calc the total number of matching
      // ids minus the number of calcs
      vector<std::unordered_set<int>> counters(calc_lookup.size());
      for(const auto& frame_info : frames) {
         Expects(frame_info.ltp_calc.size() == frame_info.calc_matches.size());
         for(size_t i = 0; i < frame_info.ltp_calc.size(); ++i) {
            const auto& ltp   = frame_info.ltp_calc[i];
            const auto& match = frame_info.calc_matches[i];
            if(match >= 0) {
               Expects(size_t(match) < frame_info.ltp_gt.size());
               Expects(calc_lookup.count(unsigned(ltp.track_id)) > 0);
               const auto tid
                   = frame_info.ltp_gt.at(size_t(match)).ltp.track_id;
               const auto index = calc_lookup[unsigned(ltp.track_id)];
               assert(index < counters.size());
               counters[index].insert(tid);
            }
         }
      }

      size_t counter = 0;
      for(const auto& set : counters) {
         const auto N = set.size();
         if(N > 1) counter += size_t(N - 1);
      }

      total_identity_switches = counter;
   }

   { // Calculate track fragmentation, mostly-traced, mostly-missed
      vector<vector<int>> gt_frames(gt_lookup.size());

      for(const auto& info : frames) {
         for(const auto& gt : info.ltp_gt) {
            if(gt.calc_index >= 0) {
               const auto index = gt_lookup.at(unsigned(gt.ltp.track_id));
               gt_frames[index].push_back(gt.ltp.tp.t);
            }
         }
      }

      // Remove duplicates and sort is required for the next two steps
      for(auto& frame_ts : gt_frames) {
         remove_duplicates(frame_ts);
         std::sort(begin(frame_ts), end(frame_ts));
      }

      // frags
      total_track_frags = 0;
      for(auto& frame_ts : gt_frames)
         for(size_t i = 1; i < frame_ts.size(); ++i)
            if(frame_ts[i - 1] + 1 != frame_ts[i]) total_track_frags++;

      //
      total_mt = 0;
      total_ml = 0;
      for(const auto& gt : ground_truth) {
         if(gt.path.size() == 0) {
            total_mt += 1;
            continue;
         }
         Expects(gt.path.front().t <= gt.path.back().t);
         const size_t len = size_t(gt.path.back().t - gt.path.front().t + 1);
         Expects(gt_lookup.count(unsigned(gt.id)) > 0);
         const size_t index = gt_lookup[unsigned(gt.id)];
         Expects(index < gt_frames.size());
         Expects(len >= gt_frames[index].size());
         const real percent = real(gt_frames[index].size()) / real(len);
         if(false) {
            INFO(format("track-id = {}, percent = {}", gt.id, percent));
            cout << format(
                "   [{}] ==> [{}]\n",
                rng::implode(
                    gt.path, ", ", [&](const auto& tp) { return str(tp.t); }),
                rng::implode(gt_frames[index], ", "))
                 << endl;
         }
         if(percent >= 0.80)
            total_mt += 1;
         else if(percent < 0.20)
            total_ml += 1;
      }
   }

   { // calcualte distances && IoU
      total_tp_distances = 0.0;
      total_tp_iou       = 0.0;

      const float r = float(track_radius);

      vector<std::unordered_set<int>> counters(calc_lookup.size());
      for(const auto& frame_info : frames) {
         for(const auto& gt : frame_info.ltp_gt) {
            if(gt.calc_index >= 0) {
               Expects(size_t(gt.calc_index) < frame_info.ltp_calc.size());
               const auto gt_xy = gt.ltp.tp.xy();
               const auto calc_xy
                   = frame_info.ltp_calc[size_t(gt.calc_index)].tp.xy();

               total_tp_distances += real((gt_xy - calc_xy).norm()) * hist_sz;
               total_tp_iou
                   += real(intersection_over_union(gt_xy, r, calc_xy, r));
            }
         }
      }
   }

   { // calculate proportion of track-points with gaze
      uint32_t counter = 0;
      uint32_t total   = 0;

      auto process_tp = [&](const TrackPoint& tp) {
         total++;
         if(std::isfinite(tp.gaze_direction)) counter++;
      };

      for(const auto& tt : calculated)
         for(const auto& tp : tt.path) process_tp(tp);

      proportion_with_gaze = (total == 0) ? 1.0 : real(counter) / real(total);
   }

   if(false) {
      INFO(format("frames [0]:\n\n{}",
                  rng::implode(frames, "\n", [&](const auto& f) {
                     return f.to_string();
                  })));
   }
}

// -------------------------------------------------------------------- total_pt
//
size_t This::total_pt() const noexcept
{
   Expects(ground_truth.size() >= (total_mt + total_ml));
   return ground_truth.size() - total_mt - total_ml;
}

// -------------------------------------------------------------------------- f1
//
real This::f1() const noexcept
{
   const auto N
       = (total_true_positives + total_false_negatives + total_false_positives);

   return (N == 0) ? 0.0 : (total_true_positives / real(N));
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   return format(R"V0G0N(
+--------------------------
|TestOutput:
|   pass:                 {}
|
|   frames:              [{}..{})
|   hist-sz:              {}
|
|   gt frame-rate         {} fps
|   calc frame-rate       {} fps
|
|   # gt-tracks:          {}
|   # detected tracks:    {}
|
|   true positives:       {}
|   false positives:      {}
|   false negatives:      {}
|   f1:                   {}
|
|   total-id-switches:    {}
|   total-frags:          {}
|
|   total-tp-distances:   {:.3f}
|   total-tp-iou:         {}
|
|   total-mostly-tracked: {}
|   total-partly-tracked: {}
|   total-mostly-lost:    {}
|
|   proportion with gaze: {}
|   average theta bias:   {}
|   theta error stddev:   {}
|   total theta errors:   {}/{} = {}%
+--------------------------

{})V0G0N",
                 str(test_passes),
                 start_frame,
                 start_frame + calc_n_frames,
                 hist_sz,
                 gt_frame_rate,
                 calc_frame_rate,
                 ground_truth.size(),
                 calculated.size(),
                 total_true_positives,
                 total_false_positives,
                 total_false_negatives,
                 f1(),
                 total_identity_switches,
                 total_track_frags,
                 total_tp_distances,
                 total_tp_iou,
                 total_mt,
                 total_pt(),
                 total_ml,
                 proportion_with_gaze,
                 to_degrees(theta_stats.average),
                 to_degrees(theta_stats.stddev),
                 total_theta_errors,
                 theta_stats.N,
                 100.0 * total_theta_errors / real(theta_stats.N),
                 "");
}

string str(const TestOutput& o) noexcept { return o.to_string(); }

// --------------------------------------------------------------------- compare

TestOutput compare_tracker_output(const FowlkesResult& fw_gt,
                                  const FowlkesResult& fw_calc,
                                  const real radius) noexcept(false)
{
   int calc_n_frames = fw_calc.frames;
   Expects(calc_n_frames >= 0);

   if(false) {
      save(fw_gt, "/tmp/zzz-0-gt.json");
      save(fw_calc, "/tmp/zzz-1-calc.json");
      FATAL(format("radius = {}", radius));
   }

   const auto gaze_theta_threshold = to_radians(45.0);

   return compare_tracker_output(0,
                                 unsigned(calc_n_frames),
                                 fw_gt.tracks,
                                 fw_gt.frame_duration,
                                 fw_calc.tracks,
                                 fw_calc.frame_duration,
                                 fw_calc.bounds(),
                                 fw_calc.hist_sz,
                                 radius,
                                 gaze_theta_threshold);
}

TestOutput
compare_tracker_output(const unsigned start_frame,
                       const unsigned calc_n_frames, // calc-n-frames
                       const vector<Track>& ground_truth,
                       const real gt_frame_duration,
                       const vector<Track>& output_data,
                       const real calc_frame_duration,
                       const AABB& hist_bounds,
                       const real hist_sz,
                       const real track_radius,
                       const real gaze_theta_threshold) noexcept(false)
{
   TestOutput ret;
   ret.init(start_frame,
            calc_n_frames,
            ground_truth,
            gt_frame_duration,
            output_data,
            calc_frame_duration,
            hist_bounds,
            hist_sz,
            track_radius,
            gaze_theta_threshold);

   return ret;
}

} // namespace perceive::pipeline
