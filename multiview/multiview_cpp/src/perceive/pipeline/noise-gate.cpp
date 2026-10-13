
#include "noise-gate.hpp"

#include "test-output.hpp"

#define This NoiseTolerances

namespace perceive::pipeline
{
// ------------------------------------------------------------------- meta data
//
const vector<MemberMetaData>& This::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(This, REAL, f1_drop_max, true));
      m.push_back(MAKE_META(This, INT, count_abs_slack, true));
      m.push_back(MAKE_META(This, REAL, count_rel_slack, true));
      m.push_back(MAKE_META(This, REAL, mean_dist_increase_max, true));
      m.push_back(MAKE_META(This, REAL, agree_precision_min, true));
      m.push_back(MAKE_META(This, REAL, agree_recall_min, true));
      m.push_back(MAKE_META(This, REAL, agree_jaccard_min, true));
      m.push_back(MAKE_META(This, REAL, agree_mean_dist_max, true));
      m.push_back(MAKE_META(This, INT, id_switch_slack, true));
      m.push_back(MAKE_META(This, INT, frag_slack, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// ------------------------------------------------------------ mean-tp-distance
//
static real mean_tp_distance(const TestOutput& o) noexcept
{
   return (o.total_true_positives == 0)
              ? 0.0
              : (o.total_tp_distances / real(o.total_true_positives));
}

// ---------------------------------------------------------------- within-noise
//
bool within_noise(const TestOutput* truth_ref,
                  const TestOutput* truth_cand,
                  const TestOutput& agreement,
                  const NoiseTolerances& tol,
                  const int n_boundaries,
                  string& report) noexcept
{
   bool all_pass = true;
   std::stringstream ss{""};

   auto clause =
       [&](const bool pass, const string_view name, const string_view detail) {
          ss << format("{} {:<13} {}\n",
                       (pass ? "pass" : "FAIL"),
                       format("{}:", name),
                       detail);
          if(!pass) all_pass = false;
       };

   if((truth_ref == nullptr) != (truth_cand == nullptr)) {
      clause(false, "truth", "need both truth scores, or neither");
   } else if(truth_ref != nullptr) {
      const auto& ref  = *truth_ref;
      const auto& cand = *truth_cand;

      clause(cand.f1() >= ref.f1() - tol.f1_drop_max,
             "f1",
             format("{:.4f} vs {:.4f}, may drop by {}",
                    cand.f1(),
                    ref.f1(),
                    tol.f1_drop_max));

      const real n_gt_points
          = real(ref.total_true_positives + ref.total_false_negatives);
      const real slack = std::max(real(tol.count_abs_slack),
                                  tol.count_rel_slack * n_gt_points);
      auto count_clause
          = [&](const string_view name, const unsigned a, const unsigned b) {
               clause(std::fabs(real(a) - real(b)) <= slack,
                      name,
                      format("{} vs {}, may differ by {:.1f}", a, b, slack));
            };
      count_clause("tp", cand.total_true_positives, ref.total_true_positives);
      count_clause("fp", cand.total_false_positives, ref.total_false_positives);
      count_clause("fn", cand.total_false_negatives, ref.total_false_negatives);

      clause(mean_tp_distance(cand)
                 <= mean_tp_distance(ref) + tol.mean_dist_increase_max,
             "mean-dist",
             format("{:.4f} vs {:.4f}, may grow by {}",
                    mean_tp_distance(cand),
                    mean_tp_distance(ref),
                    tol.mean_dist_increase_max));

      const size_t n_b       = size_t(std::max(1, n_boundaries));
      const size_t id_slack  = size_t(std::max(0, tol.id_switch_slack)) * n_b;
      const size_t frg_slack = size_t(std::max(0, tol.frag_slack)) * n_b;
      clause(cand.total_identity_switches
                 <= ref.total_identity_switches + id_slack,
             "id-switches",
             format("{} vs {}, may grow by {}",
                    cand.total_identity_switches,
                    ref.total_identity_switches,
                    id_slack));
      clause(cand.total_track_frags <= ref.total_track_frags + frg_slack,
             "frags",
             format("{} vs {}, may grow by {}",
                    cand.total_track_frags,
                    ref.total_track_frags,
                    frg_slack));
   }

   { // Agreement; two empty runs agree
      const auto& o         = agreement;
      const bool both_empty = (o.total_true_positives + o.total_false_positives
                               + o.total_false_negatives)
                              == 0;
      clause(both_empty || o.precision() >= tol.agree_precision_min,
             "agree-prec",
             format("{:.4f}, at least {}",
                    o.precision(),
                    tol.agree_precision_min));
      clause(both_empty || o.recall() >= tol.agree_recall_min,
             "agree-recall",
             format("{:.4f}, at least {}", o.recall(), tol.agree_recall_min));
      clause(both_empty || o.f1() >= tol.agree_jaccard_min,
             "agree-f1",
             format("{:.4f}, at least {}", o.f1(), tol.agree_jaccard_min));
      clause(mean_tp_distance(o) <= tol.agree_mean_dist_max,
             "agree-dist",
             format("{:.4f}, at most {}",
                    mean_tp_distance(o),
                    tol.agree_mean_dist_max));
   }

   report = ss.str();
   return all_pass;
}

} // namespace perceive::pipeline
