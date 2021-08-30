
#include "cli-args.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

#define This CliArgs

namespace perceive::pipeline
{
// ------------------------------------------------------------------- meta data
//
const vector<MemberMetaData>& This::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(This, STRING, version, true));
      m.push_back(MAKE_META(This, BOOL, show_help, true));
      m.push_back(MAKE_META(This, BOOL, has_error, true));
      m.push_back(MAKE_META(This, STRING, stats_filename, false));
      m.push_back(MAKE_META(This, STRING, tracks_filename, false));
      m.push_back(MAKE_META(This, JSON_VALUE, in_stats_filenames, false));
      m.push_back(MAKE_META(This, BOOL, allow_overwrite, false));
      m.push_back(MAKE_META(This, BOOL, generate_stats, false));
      m.push_back(MAKE_META(This, BOOL, generate_tracks, false));
      m.push_back(MAKE_META(This, INT, verbosity, false));
      m.push_back(MAKE_META(This, STRING, outdir, false));
      m.push_back(MAKE_META(This, INT, start_frame_no, true));
      m.push_back(MAKE_META(This, INT, n_frames_to_process, true));
      m.push_back(MAKE_META(This, REAL, target_fps, true));
      m.push_back(MAKE_META(This, STRING, pipeline_output_filename, false));
      m.push_back(MAKE_META(This, BOOL, debug_video_output, false));
      m.push_back(MAKE_META(This, BOOL, presentation_video_output, false));
      m.push_back(MAKE_META(This, BOOL, generate_timings, false));
      m.push_back(MAKE_META(This, BOOL, save_localization_data, false));
      m.push_back(MAKE_META(This, STRING, localization_data_fname, false));
      m.push_back(MAKE_META(This, BOOL, developer_features, false));
      m.push_back(MAKE_META(This, BOOL, on_the_fly_still, false));
      m.push_back(MAKE_META(This, BOOL, colour_floor_hist, false));
      m.push_back(MAKE_META(This, BOOL, no_stereo, false));
      m.push_back(MAKE_META(This, BOOL, export_training_data, false));
      m.push_back(MAKE_META(This, STRING, export_training_data_prefix, false));
      m.push_back(MAKE_META(This, STRING, pose_classifier, false));
      m.push_back(MAKE_META(This, STRING, tpfp_classifier, false));
      m.push_back(MAKE_META(This, STRING, annotations_directory, false));
      m.push_back(MAKE_META(This, BOOL, annotate_mode, false));
      m.push_back(MAKE_META(This, STRING, manifest_opt_search_path, false));
      m.push_back(MAKE_META(This, JSON_VALUE, manifest_params, false));
      m.push_back(MAKE_META(This, JSON_VALUE, pipeline_params, false));
      m.push_back(MAKE_META(This, INT, output_video_width, false));
      m.push_back(MAKE_META(This, INT, output_video_height, false));
      m.push_back(MAKE_META(This, BOOL, output_video_render_tracks, false));
      m.push_back(MAKE_META(This, BOOL, output_video_render_pose, false));
      m.push_back(
          MAKE_META(This, BOOL, output_video_p2ds_on_localization, false));
      m.push_back(MAKE_META(This, BOOL, output_video_blur_faces, false));
      m.push_back(MAKE_META(This, BOOL, output_video_ground_truth_only, false));
      m.push_back(MAKE_META(This, BOOL, run_pipeline_testcase, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// ------------------------------------------------------------------- show-help
//
void show_help(string argv0) noexcept
{
   CliArgs default_config;
   cout << format(R"V0G0N(

   Usage: {} [OPTIONS...] 

      --run-file <filename> Load parameters from the passed filename.
                            Overrides any other set parameters.
                            Note that a 'run-file' json file is always
                            created in the output directory.
      --ignore-run-file-env Ignore any environment variables set in run-file

      -y                    Allow overwrite of output files.   
      -d <dirname>          Directory to save output files to. Default '{}'.
      -o <filename>         Pipeline output filename. 
                            Default is '{}' in output directory.
    
      -p <filename>         Params filename.
      -m <filename>         Manifest filename. MUST be specified.
      --p-json <json-str>   Params inline JSON.
      --m-json <json-str>   Manifest inline JSON.    

      --no-tracks           Do not run the tracker. Useful for generating 
                            stats only.
      --start <integer>     Start frame. Default is {}.
      --n-frames <integer>  Number of frames to process. Default is {}, which 
                            indicates that all frames are processed.
      --target-fps <real>   Target frames-per-second to process at. Default is
                            0.0, which will use the movie's frames-per-second.

      --on-the-fly-still    Instead of loading or generating stills, the 
                            stills are generated on the fly from a few 
                            frames, and are NOT saved to disk.

      --verbosity <integer> 0, 1, or 2, with '2' being most verbose.
                            Default is {}.

      --save-loc-data       Save localization data to disk in order to quickly
                            run the tracker in some later run.
      --load-loc-data <fname> 

      --timings             Generate a timing image. Results are saved to
                            the output directory.

      --developer           Enable developer features. Developer only.

      --debug-video         Video is saved to the output directory.      
      --present-video       Video is saved to the output directory.
      --video-no-render-tracks         Do NOT render tracks.
      --video-no-render-pose           Do NOT render pose.
      --video-no-p2ds-on-localization  Do NOT p2ds on localization.
      --video-no-blur-faces            Do NOT blur faces on output videos.
      --video-gt-only       Render ground-truth ONLY.

      --tracks-file <fname> A (pipeline output) tracks file useful for 
                            generating demo/present video outputs.

      --pipeline-testcase   Run the pipeline testcase. A "--tracks-file" must be
                            supplied.

      --annotations-directory <dirname>     
                            Directory to load/save annotation data to. It can
                            be an `s3://` path.
      --annotate-mode       Don't show the `tracks` tab in the GUI.

      --export-training-data <name>
                            Training data is exported, with files prefixed by 
                            the passed `<name>`.

      --pose-classifier <key>  Pose classifier to load.
      --tpfp-classifier <key>  True/False positive classifier to load.

   Examples:

      # TBA

   About "manifest" files:

      #TBA

{})V0G0N",
                  basename(argv0),
                  default_config.outdir,
                  default_config.pipeline_output_filename,
                  default_config.start_frame_no,
                  default_config.n_frames_to_process,
                  default_config.verbosity,
                  "");
}

// ---------------------------------------------------------- parse-command-line
//
CliArgs parse_command_line(
    int argc,
    char** argv,
    std::function<bool(int argc, char** argv, int& i)> callback) noexcept
{
   CliArgs config;
   auto has_error = false;

   // (*) ---- Search for '-h/--help'
   // config.argv0 = argv[0]; // show_help needs argv0
   for(int i = 1; i < argc and !config.show_help; ++i)
      if(strcmp(argv[i], "-h") == 0 or strcmp(argv[i], "--help") == 0)
         config.show_help = true;
   if(config.show_help) return config;

   // (*) ---- Parse command line
   string run_fname         = ""s;
   string params_fname      = ""s;
   string params_json_str   = ""s;
   string manifest_fname    = ""s;
   string manifest_json_str = ""s;
   bool lazy_generate_stats = false;
   bool ignore_runfile_env  = false;

   for(int i = 1; i < argc; ++i) {
      const string arg = argv[i];
      try {
         if(arg == "--run-file"s) {
            run_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--ignore-run-file-env"s) {
            ignore_runfile_env = true;
         } else if(arg == "-y"s) {
            config.allow_overwrite = true;
         } else if(arg == "-d"s) {
            config.outdir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-o"s) {
            config.pipeline_output_filename = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-m"s or arg == "--manifest"s) {
            manifest_fname = cli::safe_arg_str(argc, argv, i);
            config.manifest_opt_search_path
                = dirname(absolute_path(manifest_fname));
         } else if(arg == "--m-json"s) {
            manifest_json_str = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-s"s or arg == "--stats"s) { // deprecated
            WARN(format("use of deprecated flag '{}'", arg));
            config.push_in_stats_fname(cli::safe_arg_str(argc, argv, i));
            config.no_stats = false;
         } else if(arg == "-g"s) { // deprecated
            WARN(format("use of deprecated flag '{}'", arg));
            config.stats_filename = cli::safe_arg_str(argc, argv, i);
            config.generate_stats = true;
            config.no_stats       = false;
         } else if(arg == "--lazy-g"s) { // deprecated
            WARN(format("use of deprecated flag '{}'", arg));
            config.stats_filename = cli::safe_arg_str(argc, argv, i);
            lazy_generate_stats   = true;
            config.no_stats       = false;
         } else if(arg == "--no-stats"s) {
            config.no_stats = true;
         } else if(arg == "--no-tracks"s) {
            config.generate_tracks = false;
         } else if(arg == "-p"s or arg == "--params"s) {
            params_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--p-json"s) {
            params_json_str = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--start"s) {
            config.start_frame_no = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--n-frames"s) {
            config.n_frames_to_process = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--target-fps"s) {
            config.target_fps = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--on-the-fly-still"s) {
            config.on_the_fly_still = true;
         } else if(arg == "--tracks-file"s) {
            config.tracks_filename = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--verbosity"s) {
            config.verbosity = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--timings"s) {
            config.generate_timings = true;
         } else if(arg == "--save-loc-data"s) {
            config.save_localization_data = true;
         } else if(arg == "--load-loc-data"s) {
            config.localization_data_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--developer"s) {
            config.developer_features = true;
         } else if(arg == "--no-stereo"s) {
            config.no_stereo = true;
         } else if(arg == "--debug-video"s) {
            config.debug_video_output = true;
         } else if(arg == "--present-video"s) {
            config.presentation_video_output = true;
         } else if(arg == "--video-width"s) {
            config.output_video_width = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--video-height"s) {
            config.output_video_height = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--video-no-render-tracks"s) {
            config.output_video_render_tracks = false;
         } else if(arg == "--video-no-render-pose"s) {
            config.output_video_render_pose = false;
         } else if(arg == "--video-no-p2ds-on-localization"s) {
            config.output_video_p2ds_on_localization = false;
         } else if(arg == "--video-no-blur-faces"s) {
            config.output_video_blur_faces = false;
         } else if(arg == "--video-gt-only"s) {
            config.output_video_ground_truth_only = true;
         } else if(arg == "--pipeline-testcase"s) {
            config.run_pipeline_testcase = true;
         } else if(arg == "--annotations-directory"s) {
            config.annotations_directory = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--annotate-mode"s) {
            config.annotate_mode = true;
         } else if(arg == "--export-training-data"s) {
            config.export_training_data = true;
            config.export_training_data_prefix
                = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--pose-classifier"s) {
            config.pose_classifier = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--tpfp-classifier"s) {
            config.tpfp_classifier = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--setup-testcase"s) {
            config.setup_testcase = true;
         } else {
            bool warn = true;
            if(callback) { warn = !callback(argc, argv, i); }
            if(warn) {
               WARN(format("Unused argument: '{}", arg));
               has_error = true;
            }
         }
      } catch(std::runtime_error& e) {
         cout << format("error on command-line: {}", e.what()) << endl;
         has_error = true;
      }
   }

   const bool has_run_fname = !run_fname.empty();
   if(has_run_fname) {
      try {
         Json::Value o = parse_json(file_get_contents(run_fname));
         if(o["args"].type() != Json::objectValue)
            throw std::runtime_error(
                "could not find json object value at key 'args'");
         if(o["env"].type() != Json::objectValue)
            throw std::runtime_error(
                "could not find json object value at key 'env'");

         CliArgs defaults;
         config.read_with_defaults(o["args"], &defaults, true);
         if(!ignore_runfile_env) set_environment_variables(o["env"]);

      } catch(std::exception& e) {
         LOG_ERR(
             format("failed to read run file '{}': {}", run_fname, e.what()));
         has_error = true;
      }
   }

   if(!is_directory(config.outdir)) {
      WARN(format("output directory '{}' doesn't exist, creating...",
                  config.outdir));
      mkdir_p(config.outdir);
   }

   if(!is_directory(config.outdir)) {
      LOG_ERR(format("failed to create directory '{}'", config.outdir));
      has_error = true;
   }

   if(config.setup_testcase) {
      config.save_localization_data = true;
      config.generate_tracks        = true;
   }

   if(!config.localization_data_fname.empty()) {
      if(!is_regular_file(config.localization_data_fname)) {
         LOG_ERR(format("failed to find localization-data file: '{}'",
                        config.localization_data_fname));
         has_error = true;
      } else {
         config.ldata_env_ptr = make_shared<LocalizationDataEnvelope>(
             load_localization_data(config.localization_data_fname));

         Expects(config.ldata_env_ptr != nullptr);
         Expects(std::isfinite(config.ldata_env_ptr->frame_duration));
         const auto& ldat = *config.ldata_env_ptr;

         // Make sure the start/n-frames is valid
         if(config.start_frame_no > int(ldat.start_frame)) {
            WARN(format("loaded localization data starts at frame {}, but "
                        "start-frame was set to {}. Setting start-frame to {}",
                        ldat.start_frame,
                        config.start_frame_no,
                        ldat.start_frame));
            config.start_frame_no = int(ldat.start_frame);
         }

         if(config.n_frames_to_process < 0) {
            config.n_frames_to_process = int(ldat.loc_data.size());
         } else if(config.n_frames_to_process > int(ldat.loc_data.size())) {
            config.n_frames_to_process = int(ldat.loc_data.size());
         }

         // The target fps
         const auto target_fps = 1.0 / ldat.frame_duration;
         if(config.target_fps == 0.0) {
            config.target_fps = target_fps;
         } else if(!is_close(target_fps, config.target_fps)) {
            WARN(format("target-fps set to '{}', but loaded localization-data "
                        "has a target-fps of {}. Using {}.",
                        config.target_fps,
                        target_fps,
                        config.target_fps));
            config.target_fps = target_fps;
         }
      }
   }

   for(const auto& fname : config.in_stats_filenames) {
      if(lazy_exists(fname.asString()) == LazyExists::NOWHERE) {
         LOG_ERR(
             format("failed to find input stats file '{}'", fname.asString()));
         has_error = true;
      }
   }

   if(lazy_generate_stats and config.n_in_stats_fnames() > 0) {
      LOG_ERR(format("cannot --lazy-g and load stats (-s <filename>) "
                     "at the same time!"));
      has_error = true;
   }

   if(config.target_fps < 0.0 or config.target_fps > 120.0
      or !std::isfinite(config.target_fps)) {
      LOG_ERR(
          format("invalid --target-fps = {}. Should be in the range [0..120]",
                 config.target_fps));
      has_error = true;
   }

   if(lazy_generate_stats) {
      // First, figure out if we've got a file...
      auto& fname         = config.stats_filename;
      const bool is_empty = fname.empty();
      const bool is_s3    = !is_empty && is_s3_uri(fname);
      const bool is_file  = !is_empty && !is_s3 && is_regular_file(fname);

      if(is_empty) {
         config.generate_stats = true;
      } else if(is_file) {
         config.push_in_stats_fname(fname);
         config.generate_stats = false;
         fname                 = ""s;
      } else if(lazy_s3_exists(fname) == LazyExists::NOWHERE) {
         config.generate_stats = true;
      } else if(is_s3) {
         config.push_in_stats_fname(fname);
         config.generate_stats = false;
         fname                 = ""s;
      } else {
         FATAL("logic error");
      }
   }

   if(config.generate_stats and config.stats_filename.empty()) {
      config.stats_filename = format("{}/stats.data", config.outdir);
   }

   if(config.generate_stats and config.n_in_stats_fnames() == 0) {
      config.push_in_stats_fname(config.stats_filename);
   }

   // (*) ---- Perform sanity checks
   if(config.generate_stats and !config.stats_filename.empty()
      and !config.allow_overwrite and is_regular_file(config.stats_filename)
      and !begins_with(config.stats_filename, config.outdir)) {
      LOG_ERR(
          format("cowardly refusing to overwrite '{}'", config.stats_filename));
      has_error = true;
   }

   if(config.pipeline_output_filename.empty()) {
      LOG_ERR(format("must specify an output filename (i.e., -o <filename>)"));
      has_error = true;
   } else if(config.pipeline_output_filename[0] != '/') {
      config.pipeline_output_filename
          = format("{}/{}", config.outdir, config.pipeline_output_filename);
   } else if(!config.allow_overwrite
             and is_regular_file(config.pipeline_output_filename)) {
      LOG_ERR(format("cowardly refusing to overwite '{}'",
                     config.pipeline_output_filename));
      has_error = true;
   }

   if(config.generate_timings and config.generate_stats) {
      // LOG_ERR(
      //     format("timings information is inaccurate when generating stats"));
      // has_error = true;
   }

   if(false) {
      if(!config.no_stats and !config.generate_stats
         and config.in_stats_filenames.empty()) {
         LOG_ERR("must either generate stats, or load stats. (Or both.)");
         has_error = true;
      }
   }

   if(config.no_stats and config.generate_stats) {
      LOG_ERR(format("cannot both generate stats, and use no stats!"));
      has_error = true;
   }

   if(!config.tracks_filename.empty()
      and !is_regular_file(config.tracks_filename)) {
      LOG_ERR(format("cannot find tracks file '{}'", config.tracks_filename));
      has_error = true;
   }

   { // pipeline testcase...
      if(config.run_pipeline_testcase and config.tracks_filename.empty()) {
         LOG_ERR(format("must specify a tracks filename when running a "
                        "pipeline testcase."));
         has_error = true;
      }
   }

   // Handle parameters
   if(!has_run_fname) {
      if(!params_json_str.empty()) {
         if(!parse_json(params_json_str, config.pipeline_params)) {
            LOG_ERR(format("failed to parse params json string: '{}'",
                           params_json_str));
            has_error = true;
         }
      } else if(!params_fname.empty()) {
         try {
            config.pipeline_params
                = parse_json(file_get_contents(params_fname));
         } catch(std::exception& e) {
            LOG_ERR(format(
                "failed to load json file '{}': {}", params_fname, e.what()));
            has_error = true;
         }
      }
   }

   // Handle manifest
   if(!has_run_fname) {
      if(!manifest_json_str.empty()) {
         if(!parse_json(manifest_json_str, config.manifest_params)) {
            LOG_ERR(format("failed to parse manifest json string: '{}'",
                           manifest_json_str));
            has_error = true;
         }
      } else if(!manifest_fname.empty()) {
         try {
            config.manifest_params
                = parse_json(file_get_contents(manifest_fname));
         } catch(std::exception& e) {
            LOG_ERR(format(
                "failed to load json file '{}': {}", manifest_fname, e.what()));
            has_error = true;
         }
      }
   }

   // Annotations directory
   if(config.annotations_directory != ""s) {
      if(is_s3_uri(config.annotations_directory)) {
         // Hope it's okay
      } else if(!is_directory(config.annotations_directory)) {
         LOG_ERR(format("failed to find annotations directory '{}'",
                        config.annotations_directory));
         has_error = true;
      }
   }

   if(config.export_training_data && config.tracks_filename.empty()) {
      LOG_ERR(
          format("must specify `--tracks-file` when exporting training data"));
      has_error = true;
   }

   // (*) ---- Finalize and return result
   config.has_error = has_error;

   if(!config.has_error) {
      load_environment_variables();
      const string fname = format("{}/run-object.json", config.outdir);
      file_put_contents(fname, config.multiview_run_object_str());
   }

   return config;
} // namespace perceive::pipeline

// ------------------------------------------------------------- in-stats-fnames
//
unsigned This::n_in_stats_fnames() const noexcept
{
   return (in_stats_filenames.type() != Json::arrayValue)
              ? 0
              : in_stats_filenames.size();
}

void This::push_in_stats_fname(const string_view fname) noexcept
{
   if(in_stats_filenames.type() != Json::arrayValue)
      in_stats_filenames = Json::Value(Json::arrayValue);
   in_stats_filenames.append(Json::Value(string(fname)));
}

string This::in_stats_fname(const unsigned ind) const noexcept
{
   if((in_stats_filenames.type() == Json::arrayValue)
      and (ind < in_stats_filenames.size()))
      return in_stats_filenames[ind].asString();
   return ""s;
}

bool This::is_loading_stats() const noexcept
{
   if(generate_stats and n_in_stats_fnames() == 1
      and in_stats_fname(0) == stats_filename)
      return false;
   return n_in_stats_fnames() > 0;
}

bool This::is_generating_stats_before_run() const noexcept
{
   return generate_stats and !is_loading_stats();
}

bool This::has_pipeline_params() const noexcept
{
   return pipeline_params != Json::nullValue;
}

bool This::has_manifest_params() const noexcept
{
   return manifest_params != Json::nullValue;
}

// ---------------------------------------------------- multiview run object str
//
string This::multiview_run_object_str() const noexcept
{
   const auto ret = format(
       R"V0G0N({{
   "version": {},
   "env":     {},
   "args":    {}
{}}})V0G0N",
       json_encode(str(k_version)),
       trim_copy(indent(str_replace("\t", "   ", environment_json_str()), 3)),
       this->to_json_string(3),
       "");
   return ret;
}

} // namespace perceive::pipeline
