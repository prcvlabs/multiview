
#include "stdinc.hpp"

#include "scene-info-inc.hpp"

#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/pipeline/pipeline-input.hpp"
#include "perceive/scene/scene-manifest.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::scene_info
{
// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "Prints the frames, cameras, sensors and fps of a scene as json.";
}

// ---------------------------------------------------------------------- config
//
struct Config
{
   bool has_error        = false;
   bool show_help        = false;
   string manifest_fname = ""s;
   string params_fname   = ""s;
   string out_fname      = ""s;
   real target_fps       = 0.0;
   bool no_stereo        = false;
};

// ------------------------------------------------------------------- show help
//
void show_help(const string_view argv0)
{
   cout << format(R"V0G0N(

   Usage: {:s} -m <manifest.json> [-p <params.json>] [-o <file.json>]
                [--target-fps <fps>] [--no-stereo]

   Loads the scene as `pipeline` does and prints one json object: the number
   of frames, the cameras and their video frame counts, the sensors, the fps,
   and K, the frames per tracklet that `pipeline -p <params.json>` would use
   (the shards of a tracks job must be aligned to K). The videos must be
   readable, and the calibration available in the asset store.

      -m <filename>        The scene manifest.
      -p <filename>        Pipeline parameters. Without it, K is the default.
      -o <filename>        Also write the json to <filename> (stdout has the
                           log lines too).
      --target-fps <fps>   As in `pipeline`; the scene's frames follow it.
      --no-stereo          As in `pipeline`.

   Exit status: 0 on success, 1 on error.

{:s})V0G0N",
                  basename(argv0),
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
         if(arg == "-m"s) {
            config.manifest_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-p"s) {
            config.params_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-o"s) {
            config.out_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--target-fps"s) {
            config.target_fps = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--no-stereo"s) {
            config.no_stereo = true;
         } else {
            LOG_ERR(format("unknown command line argument: '{}'", arg));
            config.has_error = true;
         }
      } catch(std::runtime_error& e) {
         LOG_ERR(format("Error on command-line: {}", e.what()));
         config.has_error = true;
      }
   }

   if(config.manifest_fname.empty()) {
      LOG_ERR(format("must specify a manifest (-m <filename>)"));
      config.has_error = true;
   } else if(!is_regular_file(config.manifest_fname)) {
      LOG_ERR(
          format("failed to find manifest file: '{}'", config.manifest_fname));
      config.has_error = true;
   }

   if(!config.params_fname.empty() && !is_regular_file(config.params_fname)) {
      LOG_ERR(format("failed to find params file: '{}'", config.params_fname));
      config.has_error = true;
   }

   if(config.target_fps < 0.0) {
      LOG_ERR(format("--target-fps must not be negative"));
      config.has_error = true;
   }

   return config;
}

// ------------------------------------------------------------------ tracklet-k
//
// Reads 'max_frames_per_tracklet' from `task_params` as `pipeline -p` does:
// the defaults, overwritten by 'task_params.tracklet_params', if present.
//
static unsigned tracklet_k(const Json::Value& task_params) noexcept(false)
{
   Tracklet::Params tp;
   if(task_params.type() == Json::objectValue
      && has_key(task_params, "tracklet_params"))
      tp.read_with_defaults<decltype(tp)>(
          task_params["tracklet_params"], true, "tracklet_params");
   return tp.max_frames_per_tracklet;
}

// -------------------------------------------------------------------- run-info
//
static int run_info(const Config& config) noexcept(false)
{
   const auto manifest_json
       = parse_json(file_get_contents(config.manifest_fname));
   SceneManifest manifest;
   read(manifest, manifest_json);

   Json::Value task_params{Json::nullValue};
   if(!config.params_fname.empty()) {
      pipeline::PipelineInput input;
      pipeline::PipelineInput defaults;
      input.read_with_defaults(
          parse_json(file_get_contents(config.params_fname)), &defaults);
      task_params = input.task_params;
   }
   const unsigned k = tracklet_k(task_params);

   SceneDescription::InitOptions opts;
   opts.target_fps           = config.target_fps;
   opts.optional_search_path = dirname(config.manifest_fname);
   opts.on_the_fly_still     = true;
   opts.no_stereo            = config.no_stereo;
   const auto scene_desc     = load_scene_from_manifest(manifest_json, opts);
   if(scene_desc == nullptr) {
      LOG_ERR(format("failed to load the scene of manifest '{}'",
                     config.manifest_fname));
      return EXIT_FAILURE;
   }
   const auto& sd = *scene_desc;

   Json::Value cameras{Json::arrayValue};
   for(auto i = 0; i < sd.n_cameras(); ++i) {
      Json::Value o{Json::objectValue};
      o["camera_id"]    = sd.scene_info.bcam_keys.at(size_t(i));
      o["file"]         = basename(sd.filenames.at(size_t(i)));
      o["is_video"]     = bool(sd.is_video.at(size_t(i)));
      o["frame_count"]  = sd.movie_frame_count(i);
      o["frame_offset"] = sd.movie_frame_offset(i);
      cameras.append(o);
   }

   Json::Value sensor_ids{Json::arrayValue};
   for(const auto& id : sd.sensor_ids) sensor_ids.append(id);

   Json::Value o{Json::objectValue};
   o["manifest"]       = config.manifest_fname;
   o["scene_key"]      = manifest.scene_key;
   o["n_frames"]       = sd.n_frames();
   o["n_cameras"]      = sd.n_cameras();
   o["n_sensors"]      = sd.n_sensors();
   o["sensor_ids"]     = sensor_ids;
   o["scene_fps"]      = sd.scene_fps;
   o["frame_duration"] = sd.frame_duration();
   o["movie_fps"]      = sd.movie_fps();
   o["n_movie_frames"] = sd.n_movie_frames();
   o["cameras"]        = cameras;
   o["no_stereo"]      = config.no_stereo;
   o["K"]              = k;
   o["K_from_params"]  = !config.params_fname.empty();

   cout << str(o) << endl;

   if(!config.out_fname.empty()) {
      if(const auto ec = file_put_contents_atomic(config.out_fname, str(o))) {
         LOG_ERR(format(
             "failed to write '{}': {}", config.out_fname, ec.message()));
         return EXIT_FAILURE;
      }
   }

   return EXIT_SUCCESS;
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
      return EXIT_FAILURE;
   }

   try {
      return run_info(config);
   } catch(std::exception& e) {
      LOG_ERR(format("scene_info: {}", e.what()));
   }

   return EXIT_FAILURE;
}

} // namespace perceive::scene_info
