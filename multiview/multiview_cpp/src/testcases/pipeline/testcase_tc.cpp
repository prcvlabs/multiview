
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
            [ 5, 8, 1, 0, -1, "W" ],
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

CATCH_TEST_CASE("TestcaseOutput", "[testcase-output]")
{
   const real person_radius = 0.2;

   CATCH_SECTION("testcase-output")
   {
      auto load_fw = [&](string_view s) {
         FowlkesResult fw;
         try {
            read(fw, s);
         } catch(std::exception& e) {
            FATAL(format("oops: {}", e.what()));
         }
         return fw;
      };

      const auto fw_gt   = load_fw(testcase_output_raw1_gt);
      const auto fw_calc = load_fw(testcase_output_raw1_calc);

      const auto output = compare_tracker_output(fw_gt, fw_calc, person_radius);
      cout << str(output) << endl;

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
   }
}
} // namespace perceive::pipeline
