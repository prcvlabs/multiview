
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/test-output.hpp"

namespace perceive::pipeline
{
CATCH_TEST_CASE("TestOutputJson", "[score]")
{
   CATCH_SECTION("to-json")
   {
      TestOutput o;
      o.start_frame   = 100;
      o.calc_n_frames = 499;
      o.ground_truth.resize(2);
      o.calculated.resize(5);
      o.total_true_positives    = 609;
      o.total_false_positives   = 83;
      o.total_false_negatives   = 300;
      o.total_identity_switches = 1;
      o.total_track_frags       = 17;
      o.total_tp_distances      = 123.193;
      o.total_tp_iou            = 23.25;
      o.total_mt                = 1;
      o.total_ml                = 0;
      o.proportion_with_gaze    = 0.75;
      o.theta_stats.N           = 4;
      o.theta_stats.average     = to_radians(10.0);
      o.theta_stats.stddev      = to_radians(20.0);
      o.total_theta_errors      = 3;

      const auto j = o.to_json();
      CATCH_REQUIRE(j.size() == 23);
      CATCH_REQUIRE(j["start_frame"].asInt() == 100);
      CATCH_REQUIRE(j["calc_n_frames"].asInt() == 499);
      CATCH_REQUIRE(j["n_gt_tracks"].asInt() == 2);
      CATCH_REQUIRE(j["n_calc_tracks"].asInt() == 5);
      CATCH_REQUIRE(j["tp"].asInt() == 609);
      CATCH_REQUIRE(j["fp"].asInt() == 83);
      CATCH_REQUIRE(j["fn"].asInt() == 300);
      CATCH_REQUIRE(j["id_switches"].asInt() == 1);
      CATCH_REQUIRE(j["frags"].asInt() == 17);
      CATCH_REQUIRE(j["mt"].asInt() == 1);
      CATCH_REQUIRE(j["pt"].asInt() == 1);
      CATCH_REQUIRE(j["ml"].asInt() == 0);
      CATCH_REQUIRE(j["theta_n"].asInt() == 4);
      CATCH_REQUIRE(j["theta_errors"].asInt() == 3);

      auto near = [&](const char* key, const real value) {
         return std::fabs(j[key].asDouble() - value) < 1e-9;
      };
      CATCH_REQUIRE(near("precision", 609.0 / 692.0));
      CATCH_REQUIRE(near("recall", 609.0 / 909.0));
      CATCH_REQUIRE(near("f1_jaccard", 609.0 / 992.0));
      CATCH_REQUIRE(near("tp_distance_sum", 123.193));
      CATCH_REQUIRE(near("tp_distance_mean", 123.193 / 609.0));
      CATCH_REQUIRE(near("tp_iou_sum", 23.25));
      CATCH_REQUIRE(near("proportion_with_gaze", 0.75));
      CATCH_REQUIRE(near("theta_bias", 10.0));
      CATCH_REQUIRE(near("theta_stddev", 20.0));

      // The json round-trips through text unchanged
      CATCH_REQUIRE(parse_json(str(j)) == j);
   }

   CATCH_SECTION("empty")
   {
      const TestOutput o;
      CATCH_REQUIRE(o.precision() == 0.0);
      CATCH_REQUIRE(o.recall() == 0.0);
      CATCH_REQUIRE(o.f1() == 0.0);

      const auto j = o.to_json();
      CATCH_REQUIRE(j["precision"].asDouble() == 0.0);
      CATCH_REQUIRE(j["recall"].asDouble() == 0.0);
      CATCH_REQUIRE(j["tp_distance_mean"].isNull());
      CATCH_REQUIRE(j["theta_bias"].isNull());
      CATCH_REQUIRE(j["theta_stddev"].isNull());
      CATCH_REQUIRE(parse_json(str(j)) == j);
   }

   CATCH_SECTION("f1-is-jaccard")
   {
      TestOutput o;
      o.total_true_positives  = 6;
      o.total_false_positives = 2;
      o.total_false_negatives = 4;
      CATCH_REQUIRE(std::fabs(o.precision() - 6.0 / 8.0) < 1e-12);
      CATCH_REQUIRE(std::fabs(o.recall() - 6.0 / 10.0) < 1e-12);
      CATCH_REQUIRE(std::fabs(o.f1() - 6.0 / 12.0) < 1e-12);

      o.total_false_positives = 0;
      o.total_true_positives  = 0;
      CATCH_REQUIRE(o.precision() == 0.0); // no detections
      CATCH_REQUIRE(o.recall() == 0.0);
   }
}
} // namespace perceive::pipeline
