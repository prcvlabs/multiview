#include "stdinc.hpp"

#include "ldata-inc.hpp"

#include "perceive/cost-functions/localization/envelope-ops.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::ldata
{
// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "Inspect, concatenate and compare localization-data files.";
}

// ---------------------------------------------------------------------- config
//
struct Config
{
   bool has_error         = false;
   bool show_help         = false;
   bool allow_overwrite   = false;
   bool verbose           = false;
   string op              = ""s; // info, cat or diff
   vector<string> inputs  = {};
   vector<KeepRange> keep = {};
   string out_fname       = ""s;
   string report_fname    = ""s; // cat: overlap report; diff: the result
};

// ------------------------------------------------------------------- show help
//
void show_help(const string_view argv0)
{
   cout << format(R"V0G0N(

   Usage: {:s} info <file.data>...
          {:s} cat [-y] [--keep <start>:<n>]... [--report <file.json>]
                   -o <out.data> <in.data>...
          {:s} diff [--verbose] [--json <file.json>] <a.data> <b.data>

   Localization-data files are written by `pipeline --save-loc-data`.

      info                Print a json summary of each file: frame range,
                          fps, histogram geometry, detections, size, md5,
                          and the md5 of each frame.

      cat                 Concatenate files (e.g., shards) into one, as
                          `pipeline --load-loc-data a --load-loc-data b`
                          does. The kept frame ranges must tile in order.
         --keep <s>:<n>   Keep frames [s, s+n) of the corresponding input.
                          Either none or one per input. By default each
                          input keeps the frames up to the next input's
                          start, and the last input keeps all its frames.
         --report <file>  Write the report (overlapping frames compared)
                          to <file>; it is printed in any case.
         -o <filename>    Output file.
         -y               Allow overwrite.

      diff                Compare the frames two files have in common, per
                          frame: bytes as written, histogram cells and 2D
                          poses (operator==, which has a tolerance).
         --verbose        List every frame that differs.
         --json <file>    Also write the result to <file>.

   Exit status: 0 on success; for diff, 0 if the files are identical (same
   frame range, every frame byte-equal), 1 if they differ, and 2 on error.

{:s})V0G0N",
                  basename(argv0),
                  basename(argv0),
                  basename(argv0),
                  "");
}

// ------------------------------------------------------------------ parse-keep
//
static KeepRange parse_keep(const string_view s) noexcept(false)
{
   const auto pos = s.find(':');
   if(pos == string_view::npos)
      throw std::runtime_error(
          format("expected <start>:<n> after --keep, but got '{}'", s));
   KeepRange keep;
   const string start_s  = string(s.substr(0, pos));
   const string n_s      = string(s.substr(pos + 1));
   std::size_t start_pos = 0;
   std::size_t n_pos     = 0;
   try {
      keep.start = std::stoi(start_s, &start_pos);
      keep.n     = std::stoi(n_s, &n_pos);
   } catch(std::exception&) {
      start_pos = 0;
   }
   if(start_pos != start_s.size() or n_pos != n_s.size() or n_s.empty())
      throw std::runtime_error(
          format("expected <start>:<n> after --keep, but got '{}'", s));
   return keep;
}

// ------------------------------------------------------------------ parse-args
//
Config parse_args(int argc, char** argv)
{
   Config config;

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      if(arg == "-h" or arg == "--help") config.show_help = true;
   }
   if(config.show_help) return config;

   if(argc < 2) {
      LOG_ERR(format("expected one of: info, cat, diff"));
      config.has_error = true;
      return config;
   }

   config.op = argv[1];
   if(config.op != "info"s and config.op != "cat"s and config.op != "diff"s) {
      LOG_ERR(format("unknown operation '{}': expected info, cat or diff",
                     config.op));
      config.has_error = true;
      return config;
   }

   for(int i = 2; i < argc; ++i) {
      const string_view arg = argv[i];
      try {
         if(arg == "-y"s) {
            config.allow_overwrite = true;
         } else if(arg == "-o"s) {
            config.out_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--keep"s) {
            config.keep.push_back(parse_keep(cli::safe_arg_str(argc, argv, i)));
         } else if(arg == "--report"s or arg == "--json"s) {
            config.report_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--verbose"s or arg == "-v"s) {
            config.verbose = true;
         } else if(!arg.empty() and arg[0] == '-') {
            LOG_ERR(format("unknown command line argument: '{}'", arg));
            config.has_error = true;
         } else {
            config.inputs.emplace_back(arg);
         }
      } catch(std::runtime_error& e) {
         LOG_ERR(format("Error on command-line: {}", e.what()));
         config.has_error = true;
      }
   }

   for(const auto& fname : config.inputs) {
      if(!is_regular_file(fname)) {
         LOG_ERR(format("failed to find file: '{}'", fname));
         config.has_error = true;
      }
   }

   if(config.inputs.empty()) {
      LOG_ERR(format("no input files"));
      config.has_error = true;
   }

   if(config.op == "diff"s and config.inputs.size() != 2) {
      LOG_ERR(format("diff takes 2 files, but {} were given",
                     config.inputs.size()));
      config.has_error = true;
   }

   if(config.op == "cat"s) {
      if(config.out_fname.empty()) {
         LOG_ERR(format("cat needs an output file (-o <filename>)"));
         config.has_error = true;
      } else if(!config.allow_overwrite and is_regular_file(config.out_fname)) {
         LOG_ERR(format("cowardly refusing to overwrite output file '{}'",
                        config.out_fname));
         config.has_error = true;
      }
      if(!config.keep.empty() and config.keep.size() != config.inputs.size()) {
         LOG_ERR(format("{} --keep ranges given for {} inputs",
                        config.keep.size(),
                        config.inputs.size()));
         config.has_error = true;
      }
   }

   return config;
}

// ---------------------------------------------------------------- write-report
//
static bool write_report(const string_view fname, const Json::Value& o) noexcept
{
   if(fname.empty()) return true;
   if(const auto ec = file_put_contents_atomic(fname, str(o))) {
      LOG_ERR(format("failed to write '{}': {}", fname, ec.message()));
      return false;
   }
   return true;
}

// -------------------------------------------------------------------- run-info
//
static int run_info(const Config& config) noexcept(false)
{
   for(const auto& fname : config.inputs)
      cout << str(envelope_info(load_localization_data(fname), fname)) << endl;
   return EXIT_SUCCESS;
}

// --------------------------------------------------------------------- run-cat
//
static int run_cat(const Config& config) noexcept(false)
{
   vector<LocalizationDataEnvelope> envelopes;
   envelopes.reserve(config.inputs.size());
   for(const auto& fname : config.inputs)
      envelopes.push_back(load_localization_data(fname));

   const auto ret = cat_envelopes(envelopes, config.keep);
   if(!ret.error.empty()) {
      LOG_ERR(format("cat failed: {}", ret.error));
      return EXIT_FAILURE;
   }

   const auto& env = ret.envelope;
   save_localization_data(
       config.out_fname,
       env.start_frame,
       unsigned(env.loc_data.size()),
       env.frame_duration,
       [&](unsigned t) { return &env.loc_data.at(t - env.start_frame); },
       false);

   Json::Value report{Json::objectValue};
   report["output"]      = config.out_fname;
   report["start_frame"] = int(env.start_frame);
   report["n_frames"]    = int(env.loc_data.size());
   Json::Value inputs{Json::arrayValue};
   for(const auto& fname : config.inputs) inputs.append(fname);
   report["inputs"] = inputs;
   Json::Value boundaries{Json::arrayValue};
   for(const auto& o : ret.overlap_reports) boundaries.append(o);
   report["boundaries"] = boundaries;

   cout << str(report) << endl;
   return write_report(config.report_fname, report) ? EXIT_SUCCESS
                                                    : EXIT_FAILURE;
}

// -------------------------------------------------------------------- run-diff
//
static int run_diff(const Config& config) noexcept(false)
{
   const auto a    = load_localization_data(config.inputs[0]);
   const auto b    = load_localization_data(config.inputs[1]);
   const auto diff = diff_envelopes(a, b);

   Json::Value o = diff.to_json(config.verbose);
   o["a"]        = config.inputs[0];
   o["b"]        = config.inputs[1];
   cout << str(o) << endl;

   if(!write_report(config.report_fname, o)) return 2;
   if(!diff.error.empty()) return 2;
   return diff.identical() ? EXIT_SUCCESS : 1;
}

// -------------------------------------------------------------------- run-main
//
int run_main(int argc, char** argv)
{
   const Config config = parse_args(argc, argv);
   const int error_ret = (config.op == "diff"s) ? 2 : EXIT_FAILURE;

   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      cout << format("aborting...") << endl;
      return error_ret;
   }

   try {
      if(config.op == "info"s) return run_info(config);
      if(config.op == "cat"s) return run_cat(config);
      if(config.op == "diff"s) return run_diff(config);
   } catch(std::exception& e) {
      LOG_ERR(format("ldata {}: {}", config.op, e.what()));
   }

   return error_ret;
}

} // namespace perceive::ldata
