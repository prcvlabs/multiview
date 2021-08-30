
#include "tracker-results.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/geometry/skeleton/skeleton-2d-info.hpp"

#define This TrackerResult

namespace perceive
{
const vector<MemberMetaData>& PostProcessParams::meta_data() const noexcept
{
   auto make_meta = [&]() {
      using ThisParams = PostProcessParams;
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, FLOAT, prob_fp_median, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, prob_fp_stddev, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, dist_median, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, dist_stddev, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, duration_median, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, duration_stddev, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, score_threshold, true));
      return m;
   };
   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

// ----------------------------------------------------------- to-fowlkes-result
//
FowlkesResult This::to_fowlkes_result() const noexcept
{
   FowlkesResult ret;

   ret.w              = hist_w;
   ret.h              = hist_h;
   ret.frames         = n_frames;
   ret.start_frame    = start_frame;
   ret.top_left       = top_left;
   ret.hist_sz        = hist_sz;
   ret.frame_duration = frame_duration;

   ret.tracks.resize(seqs.size());
   std::transform(
       cbegin(seqs),
       cend(seqs),
       begin(ret.tracks),
       [&](const auto& seq) -> Track { return node_sequence_to_track(seq); });

   TRACE(format("|seqs| = {}, |fp-info| = {}", seqs.size(), fp_info.size()));
   Expects(seqs.size() == fp_info.size());
   for(size_t i = 0; i < fp_info.size(); ++i)
      ret.tracks[i].fp_info = fp_info[i];

   return ret;
}

// -------------------------------------------------------- post-process-helpers
//
using tracks::NodeSequence;

static float calc_prob_fp(const NodeSequence& seq) noexcept
{
   float val   = 0.0f;
   int counter = 0;
   for(const auto& o : seq.nodes()) {
      if(!o.is_interp()) {
         val += o.prob_fp();
         ++counter;
      }
   }

   Expects(std::isfinite(val));

   return (counter == 0) ? 1.0f : (val / float(counter));
}

static float calc_dist_stddev(const TrackerResult& tr,
                              const NodeSequence& seq) noexcept
{
   if(seq.size() == 0) return 0.0f;

   auto hist_to_X = [&](const Vector2f& o) -> Vector2f {
      return to_vec2f(FloorHistogram::unproject_hist_xy(
          to_vec2(o), tr.top_left, tr.hist_sz));
   };

   auto calc_track_center = [&]() -> Vector2f {
      Vector2f sum = {0.0f, 0.0f};
      for(const auto& o : seq.nodes()) sum += hist_to_X(o.xy());
      return sum / float(seq.size());
   };

   const Vector2f C = calc_track_center();

   float sum_sq = 0.0f;
   for(const auto& o : seq.nodes())
      sum_sq = (hist_to_X(o.xy()) - C).quadrance();
   const float stddev = std::sqrt(sum_sq / float(seq.size()));

   return stddev;
}

static float calc_lab_score(const NodeSequence& seq,
                            const PostProcessParams& params) noexcept
{
   float sum   = 0.0f;
   int counter = 0;
   for(const auto& node : seq.nodes()) {
      if(std::isfinite(node.still_score())) {
         sum += node.still_score();
         ++counter;
      }
   }

   return (counter == 0) ? params.score_threshold : sum / float(counter);
}

struct SeqFalsePositiveInfo
{
   float prob_fp         = fNAN;
   float distance_stddev = fNAN;
   float LAB_score       = fNAN;
   float duration        = fNAN;

   float prob_fp_p  = fNAN;
   float dist_p     = fNAN;
   float lab_p      = fNAN;
   float duration_p = fNAN;

   float score = fNAN;
   float s2    = fNAN;

   bool is_true_positive = false;

   SeqFalsePositiveInfo() = default;
   SeqFalsePositiveInfo(const TrackerResult& tr,
                        const PostProcessParams& params,
                        const NodeSequence* ptr)
   {
      init(tr, params, ptr);
   }

   void init(const TrackerResult& tr,
             const PostProcessParams& params,
             const NodeSequence* ptr) noexcept
   {
      // -- How to Filter False Positive Tracks --
      // (*) The individual nodes are high prob_fp
      // (*) The track doesn't travel very far
      // (*) The track blends in with the background :: we're not doing this
      // (*) The track is short

      Expects(ptr != nullptr);
      const auto& seq = *ptr;

      prob_fp         = calc_prob_fp(seq);
      distance_stddev = calc_dist_stddev(tr, seq);
      LAB_score       = calc_lab_score(seq, params);
      duration        = float(tr.frame_duration * seq.n_frames());

      prob_fp_p  = float(calc_prob_fp_p(params));
      dist_p     = float(calc_dist_p(params));
      lab_p      = float(calc_lab_p(params));
      duration_p = float(calc_duration_p(params));

      score = std::cbrt(prob_fp_p * dist_p * duration_p);
      s2    = (prob_fp_p + dist_p + duration_p);

      is_true_positive = (score < params.score_threshold);
   }

   real calc_prob_fp_p(const PostProcessParams& params) const noexcept
   {
      const auto z = (prob_fp - params.prob_fp_median) / params.prob_fp_stddev;
      return real(phi_function(z));
   }

   real calc_dist_p(const PostProcessParams& params) const noexcept
   {
      const auto z
          = (distance_stddev - params.dist_median) / params.dist_stddev;
      return 1.0 - real(phi_function(z));
   }

   real calc_lab_p(const PostProcessParams& params) const noexcept
   {
      const float lab_z
          = float((real(LAB_score) - cie1976_mean) / cie1976_stddev);
      const auto lab_cdf = 1.0 - real(phi_function(lab_z));
      return std::clamp<real>(lab_cdf, 0.0, 1.0);
   }

   real calc_duration_p(const PostProcessParams& params) const noexcept
   {
      const auto z
          = (duration - params.duration_median) / params.duration_stddev;
      return 1.0 - real(phi_function(z));
   }

   string to_string() const noexcept
   {
      return format(
          "fp={:5.3f}/{:5.3f}, ds={:5.3f}/{:5.3f}, lab={:5.3f}/{:5.3f}, "
          "dur={:5.1f}s/{:5.3f}, score={:6.4f},{:6.4f}, is_tp={}",
          prob_fp,
          prob_fp_p,
          distance_stddev,
          dist_p,
          LAB_score,
          lab_p,
          duration,
          duration_p,
          score,
          s2,
          str(is_true_positive));
   }
};

TrackFalsePositiveInfo calc_fp_info(const TrackerResult& tr,
                                    const PostProcessParams& params,
                                    const NodeSequence* ptr)
{
   const auto sfpi = SeqFalsePositiveInfo{tr, params, ptr};
   TrackFalsePositiveInfo fp_info;
   fp_info.prob_fp_p        = sfpi.prob_fp_p;
   fp_info.dist_p           = sfpi.dist_p;
   fp_info.lab_p            = sfpi.lab_p;
   fp_info.duration_p       = sfpi.duration_p;
   fp_info.score            = sfpi.score;
   fp_info.is_true_positive = sfpi.is_true_positive;
   return fp_info;
}

// ---------------------------------------------------------------- post-process
//
TrackerResult post_process(const TrackerResult& tr,
                           const PostProcessParams& params) noexcept
{
   TrackerResult out = tr;

   struct SortStruct
   {
      const NodeSequence* seq;
      TrackFalsePositiveInfo fp_info;
   };

   // Filter out false-positive tracks
   vector<SortStruct> ss;
   ss.resize(out.seqs.size());
   std::transform(
       cbegin(tr.seqs), cend(tr.seqs), begin(ss), [&](const auto& seq) {
          SortStruct o;
          o.seq     = &seq;
          o.fp_info = calc_fp_info(tr, params, &seq);
          return o;
       });

   auto ii = std::partition(begin(ss), end(ss), [&](const auto& o) {
      return o.fp_info.is_true_positive;
   });
   ss.erase(ii, end(ss));

   // Sort by id again
   std::sort(begin(ss), end(ss), [&](const auto& A, const auto& B) {
      return A.seq->id() < B.seq->id();
   });

   // Copy node-sequences to output
   out.seqs.resize(ss.size());
   std::transform(cbegin(ss), cend(ss), begin(out.seqs), [&](const auto& o) {
      return *o.seq;
   });

   // Copy false-positive-info to output
   out.fp_info.resize(ss.size());
   std::transform(cbegin(ss), cend(ss), begin(out.fp_info), [&](const auto& o) {
      return o.fp_info;
   });

   if(multiview_trace_mode()) {
      auto print_summary = [&](const TrackerResult& tr) -> string {
         return indent(
             implode(
                 cbegin(tr.seqs),
                 cend(tr.seqs),
                 ",\n",
                 [&](const auto& seq) {
                    const auto sfpi = SeqFalsePositiveInfo{tr, params, &seq};
                    return format(
                        "#{:-3d}, [{:4d}..{:4d}), {}",
                        seq.id(),
                        (seq.size() == 0 ? -1 : seq.nodes().front().t()),
                        (seq.size() == 0 ? -1 : seq.nodes().back().t() + 1),
                        sfpi.to_string());
                 }),
             3);
      };
      TRACE(format(
          "PRE\n{}\n\nPOST\n{}\n\n", print_summary(tr), print_summary(out)));
   }

   return out;
}

} // namespace perceive
