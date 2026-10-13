
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/noise-gate.hpp"
#include "perceive/pipeline/test-output.hpp"

namespace perceive::pipeline
{
// ----------------------------------------------------------------- make-output
//
static TestOutput make_output(const unsigned tp,
                              const unsigned fp,
                              const unsigned fn,
                              const real tp_distances,
                              const size_t id_switches,
                              const size_t frags)
{
   TestOutput o;
   o.total_true_positives    = tp;
   o.total_false_positives   = fp;
   o.total_false_negatives   = fn;
   o.total_tp_distances      = tp_distances;
   o.total_identity_switches = id_switches;
   o.total_track_frags       = frags;
   return o;
}

CATCH_TEST_CASE("WithinNoise", "[within-noise]")
{
   // The batch run of testcase 007 against its ground truth, and a candidate
   // that agrees with it point for point.
   const auto ref       = make_output(609, 83, 300, 123.193, 0, 17);
   const auto agreement = make_output(692, 0, 0, 0.0, 0, 0);
   const NoiseTolerances tol;

   auto test = [&](const TestOutput& cand,
                   const TestOutput& agree,
                   const int n_boundaries,
                   string& report) {
      return within_noise(&ref, &cand, agree, tol, n_boundaries, report);
   };

   auto failed = [](const string& report, const string_view clause) {
      return report.find(format("FAIL {}:", clause)) != string::npos;
   };

   CATCH_SECTION("equal")
   {
      string report;
      CATCH_REQUIRE(test(ref, agreement, 1, report));
      CATCH_REQUIRE(report.find("FAIL") == string::npos);
   }

   CATCH_SECTION("f1")
   {
      string report;
      const auto cand = make_output(579, 83, 330, 117.0, 0, 17); // f1 -0.03
      CATCH_REQUIRE(!test(cand, agreement, 1, report));
      CATCH_REQUIRE(failed(report, "f1"));
   }

   CATCH_SECTION("counts")
   {
      string report;
      const auto cand = make_output(609, 103, 300, 123.193, 0, 17);
      CATCH_REQUIRE(!test(cand, agreement, 1, report));
      CATCH_REQUIRE(failed(report, "fp"));
      CATCH_REQUIRE(!failed(report, "tp"));

      // The slack is 1% of the ground-truth points: 9 on testcase 007
      const auto cand9 = make_output(609, 92, 300, 123.193, 0, 17);
      CATCH_REQUIRE(test(cand9, agreement, 1, report));
      const auto cand10 = make_output(609, 93, 300, 123.193, 0, 17);
      CATCH_REQUIRE(!test(cand10, agreement, 1, report));
   }

   CATCH_SECTION("mean-distance")
   {
      string report;
      const auto cand = make_output(609, 83, 300, 609.0 * 0.23, 0, 17);
      CATCH_REQUIRE(!test(cand, agreement, 1, report));
      CATCH_REQUIRE(failed(report, "mean-dist"));
   }

   CATCH_SECTION("agreement")
   {
      string report;
      const auto agree = make_output(985, 15, 0, 0.0, 0, 0);
      CATCH_REQUIRE(!test(ref, agree, 1, report));
      CATCH_REQUIRE(failed(report, "agree-prec"));
      CATCH_REQUIRE(!failed(report, "agree-recall"));
      CATCH_REQUIRE(!failed(report, "agree-f1"));

      const auto far = make_output(692, 0, 0, 692.0 * 0.04, 0, 0);
      CATCH_REQUIRE(!test(ref, far, 1, report));
      CATCH_REQUIRE(failed(report, "agree-dist"));
   }

   CATCH_SECTION("boundaries")
   {
      string report;
      auto with_frags = [&](const size_t frags, const int n_boundaries) {
         const auto cand = make_output(609, 83, 300, 123.193, 0, frags);
         return test(cand, agreement, n_boundaries, report);
      };
      CATCH_REQUIRE(with_frags(17 + 2, 1));
      CATCH_REQUIRE(!with_frags(17 + 3, 1));
      CATCH_REQUIRE(failed(report, "frags"));
      CATCH_REQUIRE(with_frags(17 + 4, 2));
      CATCH_REQUIRE(!with_frags(17 + 5, 2));
      CATCH_REQUIRE(with_frags(17 + 2, 0)); // 0 boundaries count as 1

      auto with_ids = [&](const size_t ids, const int n_boundaries) {
         const auto cand = make_output(609, 83, 300, 123.193, ids, 17);
         return test(cand, agreement, n_boundaries, report);
      };
      CATCH_REQUIRE(with_ids(1, 1));
      CATCH_REQUIRE(!with_ids(2, 1));
      CATCH_REQUIRE(failed(report, "id-switches"));
      CATCH_REQUIRE(with_ids(2, 2));
   }

   CATCH_SECTION("no-ground-truth")
   {
      string report;
      CATCH_REQUIRE(within_noise(nullptr, nullptr, agreement, tol, 1, report));
      CATCH_REQUIRE(report.find("mean-dist:") == string::npos);
      CATCH_REQUIRE(report.find("frags:") == string::npos);
      CATCH_REQUIRE(report.find("agree-f1:") != string::npos);

      const auto agree = make_output(600, 92, 0, 0.0, 0, 0);
      CATCH_REQUIRE(!within_noise(nullptr, nullptr, agree, tol, 1, report));

      // Both truth scores, or neither
      CATCH_REQUIRE(!within_noise(&ref, nullptr, agreement, tol, 1, report));
      CATCH_REQUIRE(failed(report, "truth"));

      // Two runs without any tracks agree
      const TestOutput empty;
      CATCH_REQUIRE(within_noise(nullptr, nullptr, empty, tol, 1, report));
   }

   CATCH_SECTION("tolerances-json")
   {
      NoiseTolerances a;
      a.f1_drop_max            = 0.5;
      a.count_abs_slack        = 7;
      a.count_rel_slack        = 0.25;
      a.mean_dist_increase_max = 0.125;
      a.agree_precision_min    = 0.5;
      a.agree_recall_min       = 0.25;
      a.agree_jaccard_min      = 0.75;
      a.agree_mean_dist_max    = 0.0625;
      a.id_switch_slack        = 3;
      a.frag_slack             = 4;

      string s;
      write(a, s);
      NoiseTolerances b;
      read(b, s);
      CATCH_REQUIRE(b.f1_drop_max == a.f1_drop_max);
      CATCH_REQUIRE(b.count_abs_slack == a.count_abs_slack);
      CATCH_REQUIRE(b.count_rel_slack == a.count_rel_slack);
      CATCH_REQUIRE(b.mean_dist_increase_max == a.mean_dist_increase_max);
      CATCH_REQUIRE(b.agree_precision_min == a.agree_precision_min);
      CATCH_REQUIRE(b.agree_recall_min == a.agree_recall_min);
      CATCH_REQUIRE(b.agree_jaccard_min == a.agree_jaccard_min);
      CATCH_REQUIRE(b.agree_mean_dist_max == a.agree_mean_dist_max);
      CATCH_REQUIRE(b.id_switch_slack == a.id_switch_slack);
      CATCH_REQUIRE(b.frag_slack == a.frag_slack);
      CATCH_REQUIRE(a == b);
   }
}
} // namespace perceive::pipeline
