
#include <algorithm>
#include <future>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/test-output.hpp"

namespace perceive::pipeline
{
static const bool feedback = false;

static constexpr string_view testcase_output_raw1_gt = R"V0G0N(
{
   "frame-duration" : 0.066666666666666666,
   "start-frame" : 0,
   "frames" : 4,
   "height" : 10,
   "width" : 10,
   "hist-sz" : 0.10,
   "top-left" : [ -3.0, -2.0 ],
   "tracks" : [

      {
         "id" : 1,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "path" : [
            [ 1.1, 3, 0, 0, -1, "W" ],
            [ 1.9, 2, 1, 0, -1, "W" ],
            [ 2.9, 2, 2, 0, -1, "W" ],
            [ 4, 2, 3, 0, -1, "W" ]
         ]
      },

      {
         "id" : 2,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "path" : [
            [ 0, 7, 0, 0, -1, "W" ],
            [ 1, 8, 1, 0, -1, "W" ],
            [ 2, 7, 2, 0, -1, "W" ],
            [ 3, 6, 3, 0, -1, "W" ]
         ]
      },

      {
         "id" : 3,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "path" : [
            [ 7, 1, 0, 0, -1, "W" ],
            [ 7, 2, 1, 0, -1, "W" ],
            [ 8, 3, 2, 0, -1, "W" ],
            [ 8, 4, 3, 0, -1, "W" ]
         ]
      },

      {
         "id" : 4,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "path" : [
            [ 5, 1, 0, 0, -1, "W" ],
            [ 5, 2, 1, 0, -1, "W" ],
            [ 6, 3, 2, 0, -1, "W" ],
            [ 6, 4, 3, 0, -1, "W" ]
         ]
      }
   ]
}
)V0G0N";

static constexpr string_view testcase_output_raw1_calc = R"V0G0N(
{
   "frame-duration" : 0.066666666666666666,
   "start-frame" : 0,
   "frames" : 4,
   "height" : 10,
   "width" : 10,
   "hist-sz" : 0.10,
   "top-left" : [ -3.0, -2.0 ],
   "tracks" : [

      {
         "id" : 1,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "comment" : "FM = 1, 3 TP",
         "path" : [
            [ 1, 3, 0, 0, -1, "W" ],
            [ 3, 2, 2, 0, -1, "W" ],
            [ 4, 2, 3, 0, -1, "W" ]
         ]
      },

      {
         "id" : 2,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "comment" : "4 FP",
         "path" : [
            [ 4, 9, 0, 0, -1, "W" ],
            [ 5.01, 8, 1, 0, -1, "W" ],
            [ 6, 9, 2, 0, -1, "W" ],
            [ 7, 9, 3, 0, -1, "W" ]
         ]
      },

      {
         "id" : 3,
         "cost" : 0.0,
         "fp_info" : {
            "dist_p" : null,
            "duration_p" : null,
            "is_true_positive" : false,
            "lab_p" : null,
            "prob_fp_p" : null,
            "score" : null
         },
         "height" : 0.0,
         "label" : "UNLABELLED",
         "comment" : "1, id-switch, 4 TP",
         "path" : [
            [ 7, 1, 0, 0, -1, "W" ],
            [ 7, 2, 1, 0, -1, "W" ],
            [ 6, 3, 2, 0, -1, "W" ],
            [ 6, 4, 3, 0, -1, "W" ]
         ]
      }
   ]
}
)V0G0N";

// ---------------------------------------------------------------- shift-frames
//
static string shift_frames(const string_view raw, const int offset)
{
   Json::Value o    = parse_json(string(raw));
   o["start-frame"] = o["start-frame"].asInt() + offset;
   for(auto& track : o["tracks"])
      for(auto& tp : track["path"]) tp[2] = tp[2].asInt() + offset;
   return str(o);
}

CATCH_TEST_CASE("TestcaseOutput", "[testcase-output]")
{
   const real person_radius = 0.2;

   auto load_fw = [&](string_view s) {
      FowlkesResult fw;
      try {
         read(fw, s);
      } catch(std::exception& e) {
         FATAL(format("oops: {}", e.what()));
      }
      return fw;
   };

   // The counts are worked out by hand from the tracks above
   auto check_counts = [&](const TestOutput& output) {
      CATCH_REQUIRE(output.total_true_positives == 7);
      CATCH_REQUIRE(output.total_false_positives == 4);
      CATCH_REQUIRE(output.total_false_negatives == 9);
      CATCH_REQUIRE(output.total_identity_switches == 1);
      CATCH_REQUIRE(output.total_track_frags == 1);
      CATCH_REQUIRE(output.total_mt == 0);
      CATCH_REQUIRE(output.total_pt() == 3);
      CATCH_REQUIRE(output.total_ml == 1);
      CATCH_REQUIRE(std::fabs(output.total_tp_distances - 0.02) < 1e-6);
      CATCH_REQUIRE(std::fabs(output.f1() - 7.0 / 20.0) < 1e-9);
      CATCH_REQUIRE(std::fabs(output.precision() - 7.0 / 11.0) < 1e-9);
      CATCH_REQUIRE(std::fabs(output.recall() - 7.0 / 16.0) < 1e-9);
   };

   CATCH_SECTION("testcase-output")
   {
      const auto fw_gt   = load_fw(testcase_output_raw1_gt);
      const auto fw_calc = load_fw(testcase_output_raw1_calc);

      const auto output = compare_tracker_output(fw_gt, fw_calc, person_radius);
      if(feedback) cout << str(output) << endl;

      // MOTP: total-tp-distance / n-tps
      // MOTA: (n-fn + n-fp + ID) / total-gt-track-points
      // MODP: total-IoU for true-positives / n-tps
      // MODA: (n-fn / N) + (n-fp / N)
      // MT%
      // PT%
      // ML%
      // ID:   ID switches
      // FM:   Fragmentations
      // TP
      // FP
      // FN
      check_counts(output);

      const auto o = output.to_json();
      CATCH_REQUIRE(o["start_frame"].asInt() == 0);
      CATCH_REQUIRE(o["calc_n_frames"].asInt() == 4);
      CATCH_REQUIRE(o["n_gt_tracks"].asInt() == 4);
      CATCH_REQUIRE(o["n_calc_tracks"].asInt() == 3);
      CATCH_REQUIRE(o["tp"].asInt() == 7);
      CATCH_REQUIRE(o["fp"].asInt() == 4);
      CATCH_REQUIRE(o["fn"].asInt() == 9);
      CATCH_REQUIRE(o["id_switches"].asInt() == 1);
      CATCH_REQUIRE(o["frags"].asInt() == 1);
      CATCH_REQUIRE(std::fabs(o["precision"].asDouble() - 7.0 / 11.0) < 1e-9);
      CATCH_REQUIRE(std::fabs(o["recall"].asDouble() - 7.0 / 16.0) < 1e-9);
      CATCH_REQUIRE(std::fabs(o["f1_jaccard"].asDouble() - 0.35) < 1e-9);
      CATCH_REQUIRE(std::fabs(o["tp_distance_mean"].asDouble() - 0.02 / 7.0)
                    < 1e-6);
   }

   CATCH_SECTION("testcase-output-start-frame")
   {
      // The same tracks, 100 frames later, scored from frame 100
      const auto fw_gt = load_fw(shift_frames(testcase_output_raw1_gt, 100));
      const auto fw_calc
          = load_fw(shift_frames(testcase_output_raw1_calc, 100));
      CATCH_REQUIRE(fw_calc.start_frame == 100);

      const auto output = compare_tracker_output(unsigned(fw_calc.start_frame),
                                                 unsigned(fw_calc.frames),
                                                 fw_gt.tracks,
                                                 fw_gt.frame_duration,
                                                 fw_calc.tracks,
                                                 fw_calc.frame_duration,
                                                 fw_calc.bounds(),
                                                 fw_calc.hist_sz,
                                                 person_radius,
                                                 to_radians(45.0));
      check_counts(output);
      CATCH_REQUIRE(output.to_json()["start_frame"].asInt() == 100);
   }
}
} // namespace perceive::pipeline
