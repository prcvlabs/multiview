
#include "stdinc.hpp"

#include "score-inc.hpp"

#include "perceive/pipeline/noise-gate.hpp"
#include "perceive/pipeline/pipeline-input.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/pipeline/test-output.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::score
{
using pipeline::NoiseTolerances;
using pipeline::TestOutput;

constexpr int k_exit_fail  = 1; // not within noise
constexpr int k_exit_error = 2;

// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "Scores tracks against ground truth, or against another run.";
}

// ---------------------------------------------------------------------- config
//
struct Config
{
   bool has_error          = false;
   bool show_help          = false;
   string calc_fname       = ""s;
   string gt_fname         = ""s;
   string ref_fname        = ""s;
   string json_fname       = ""s;
   string tolerances_fname = ""s;
   real radius             = dNAN; // NAN => from the params of `calc`
   int start_frame         = -1;   // -1 => from `calc`
   int n_frames            = -1;   // -1 => from `calc`
   bool within_noise       = false;
   int n_boundaries        = 1;
};

// ------------------------------------------------------------------- show help
//
void show_help(const string_view argv0)
{
   cout << format(R"V0G0N(

   Usage: {:s} --calc <pipeline-output.json> --gt <ground-truth.json>
          {:s} --calc <pipeline-output.json> --ref <pipeline-output.json>
                [--gt <ground-truth.json>] [--within-noise]
                [--tolerances <file.json>] [--boundaries <n>]

   Scores the tracks of `calc` the way `pipeline --pipeline-testcase` does,
   and prints the result. The ground truth may be any pipeline-output.json.

      --calc <file>        The run to score.
      --gt <file>          The ground truth.
      --ref <file>         A reference run: scores the agreement of `calc`
                           with it, taking `ref` as the ground truth.
      --radius <metres>    Person radius. Default from the params of `calc`.
      --start <frame>      First frame to score. Default from `calc`.
      --n-frames <n>       Number of frames to score. Default from `calc`.
      --json <file>        Also write the scores to <file>: with --gt alone,
                           the score of `calc`; with --ref, an object with
                           "agreement", "truth" and "truth_ref" (with --gt),
                           and "within_noise".
      --within-noise       With --ref: test that `calc` is within noise of
                           `ref` (agreement, and with --gt the truth scores).
      --tolerances <file>  The tolerances for --within-noise, as json.
      --boundaries <n>     Number of shard boundaries in the frames scored;
                           scales the id-switch and fragmentation slack.
                           Default is 1.

   Exit status: 0 on success (and within noise), 1 if not within noise, and
   2 on error.

   Default tolerances:

{:s}

{:s})V0G0N",
                  basename(argv0),
                  basename(argv0),
                  NoiseTolerances{}.to_json_string(),
                  "");
}

// ------------------------------------------------------------------ parse-args
//
Config parse_args(int argc, char** argv)
{
   Config config;

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      if(arg == "-h" || arg == "--help") config.show_help = true;
   }
   if(config.show_help) return config;

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      try {
         if(arg == "--calc"s) {
            config.calc_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--gt"s) {
            config.gt_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--ref"s) {
            config.ref_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--json"s) {
            config.json_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--tolerances"s) {
            config.tolerances_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--radius"s) {
            config.radius = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--start"s) {
            config.start_frame = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--n-frames"s) {
            config.n_frames = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--within-noise"s) {
            config.within_noise = true;
         } else if(arg == "--boundaries"s) {
            config.n_boundaries = cli::safe_arg_int(argc, argv, i);
         } else {
            LOG_ERR(format("unknown command line argument: '{}'", arg));
            config.has_error = true;
         }
      } catch(std::runtime_error& e) {
         LOG_ERR(format("Error on command-line: {}", e.what()));
         config.has_error = true;
      }
   }

   auto check_file = [&](const string_view what, const string& fname) {
      if(!fname.empty() && !is_regular_file(fname)) {
         LOG_ERR(format("failed to find {} file: '{}'", what, fname));
         config.has_error = true;
      }
   };
   check_file("calc", config.calc_fname);
   check_file("ground-truth", config.gt_fname);
   check_file("reference", config.ref_fname);
   check_file("tolerances", config.tolerances_fname);

   if(config.calc_fname.empty()) {
      LOG_ERR(format("must specify --calc <pipeline-output.json>"));
      config.has_error = true;
   }

   if(config.gt_fname.empty() && config.ref_fname.empty()) {
      LOG_ERR(format("must specify --gt <file>, or --ref <file>, or both"));
      config.has_error = true;
   }

   if(config.within_noise && config.ref_fname.empty()) {
      LOG_ERR(format("--within-noise needs a reference run (--ref <file>)"));
      config.has_error = true;
   }

   if(!config.tolerances_fname.empty() && !config.within_noise) {
      WARN(format("--tolerances without --within-noise is ignored"));
   }

   if(std::isfinite(config.radius) && config.radius <= 0.0) {
      LOG_ERR(format("--radius must be positive, but got {}", config.radius));
      config.has_error = true;
   }

   if(config.n_boundaries < 0) {
      LOG_ERR(
          format("--boundaries must be >= 0, but got {}", config.n_boundaries));
      config.has_error = true;
   }

   return config;
}

// ---------------------------------------------------------------- load-fowlkes
//
static FowlkesResult load_fowlkes(const string_view fname,
                                  Json::Value* params) noexcept(false)
{
   PipelineOutput output;
   load(output, string(fname));
   FowlkesResult ret;
   read(ret, output.track_results);
   if(params != nullptr) *params = output.params;
   return ret;
}

// --------------------------------------------------------------- score-against
//
// Scores `calc` with `truth` as the ground truth, over [start, start + n).
//
static TestOutput score_against(const FowlkesResult& truth,
                                const FowlkesResult& calc,
                                const int start_frame,
                                const int n_frames,
                                const real radius) noexcept(false)
{
   const auto gaze_theta_threshold = to_radians(45.0);
   return pipeline::compare_tracker_output(unsigned(start_frame),
                                           unsigned(n_frames),
                                           truth.tracks,
                                           truth.frame_duration,
                                           calc.tracks,
                                           calc.frame_duration,
                                           calc.bounds(),
                                           calc.hist_sz,
                                           radius,
                                           gaze_theta_threshold);
}

// --------------------------------------------------------------------- run-all
//
static int run_all(const Config& config) noexcept(false)
{
   Json::Value calc_params;
   const auto calc = load_fowlkes(config.calc_fname, &calc_params);

   const real radius = std::isfinite(config.radius)
                           ? config.radius
                           : pipeline::lookup_person_radius(calc_params);
   const int start_frame
       = (config.start_frame >= 0) ? config.start_frame : calc.start_frame;
   const int n_frames = (config.n_frames >= 0) ? config.n_frames : calc.frames;
   if(start_frame < 0 || n_frames < 0) {
      LOG_ERR(format(
          "invalid frame range: start {}, n-frames {}", start_frame, n_frames));
      return k_exit_error;
   }

   const bool has_gt  = !config.gt_fname.empty();
   const bool has_ref = !config.ref_fname.empty();

   const auto gt
       = has_gt ? load_fowlkes(config.gt_fname, nullptr) : FowlkesResult{};
   const auto ref
       = has_ref ? load_fowlkes(config.ref_fname, nullptr) : FowlkesResult{};

   auto make_score = [&](const FowlkesResult& truth, const FowlkesResult& o) {
      return std::make_unique<TestOutput>(
          score_against(truth, o, start_frame, n_frames, radius));
   };

   const auto truth     = has_gt ? make_score(gt, calc) : nullptr;
   const auto truth_ref = (has_gt && has_ref) ? make_score(gt, ref) : nullptr;
   const auto agreement = has_ref ? make_score(ref, calc) : nullptr;

   if(truth != nullptr) {
      INFO(format("calc vs ground truth:"));
      cout << str(*truth) << endl;
   }
   if(truth_ref != nullptr) {
      INFO(format("reference vs ground truth:"));
      cout << str(*truth_ref) << endl;
   }
   if(agreement != nullptr) {
      INFO(format("calc vs reference (agreement):"));
      cout << str(*agreement) << endl;
   }

   int ret = EXIT_SUCCESS;
   Json::Value o{Json::objectValue};
   if(!has_ref) {
      o = truth->to_json();
   } else {
      o["agreement"] = agreement->to_json();
      if(truth != nullptr) o["truth"] = truth->to_json();
      if(truth_ref != nullptr) o["truth_ref"] = truth_ref->to_json();
   }

   if(config.within_noise) {
      NoiseTolerances tol;
      if(!config.tolerances_fname.empty()) load(tol, config.tolerances_fname);

      string report   = ""s;
      const bool pass = pipeline::within_noise(truth_ref.get(),
                                               truth.get(),
                                               *agreement,
                                               tol,
                                               config.n_boundaries,
                                               report);
      cout << format("within-noise ({} boundaries): {}\n{}",
                     config.n_boundaries,
                     (pass ? "pass" : "FAIL"),
                     report)
           << endl;

      Json::Value w{Json::objectValue};
      w["pass"]         = pass;
      w["n_boundaries"] = config.n_boundaries;
      w["tolerances"]   = tol.to_json();
      w["report"]       = report;
      o["within_noise"] = w;
      if(!pass) ret = k_exit_fail;
   }

   if(!config.json_fname.empty()) {
      if(const auto ec = file_put_contents_atomic(config.json_fname, str(o))) {
         LOG_ERR(format(
             "failed to write '{}': {}", config.json_fname, ec.message()));
         return k_exit_error;
      }
   }

   return ret;
}

// -------------------------------------------------------------------- run-main
//
int run_main(int argc, char** argv)
{
   const Config config = parse_args(argc, argv);

   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      cout << format("aborting...") << endl;
      return k_exit_error;
   }

   try {
      return run_all(config);
   } catch(std::exception& e) {
      LOG_ERR(format("score: {}", e.what()));
   }

   return k_exit_error;
}

} // namespace perceive::score
