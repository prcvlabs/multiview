
#include "stdinc.hpp"

#include "render-floor-map-inc.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/movie/presentation-movie.hpp"
#include "perceive/pipeline/execute.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::render_floor_map
{
// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "Create a floor map from the 3D point cloud.";
}

// ---------------------------------------------------------------------- config
//
struct Config
{
   bool has_error           = false;
   bool show_help           = false;
   string params_fname      = ""s;
   string params_json_str   = ""s;
   string manifest_fname    = ""s;
   string manifest_json_str = ""s;
   string outdir            = "/tmp"s;
   bool is_verbose          = false;
};

// ------------------------------------------------------------------- show help
//
void show_help(string argv0)
{
   Config default_config;

   cout << format(R"V0G0N(

   Usage: {:s} [OPTIONS...]

      -m <filename>       Manifest filename
      -p <filename>       Parameters file to use. Does not need to be specified.
      --m-json <json-str> Manifest filename
      --p-json <json-str> Params inline JSON.
      -d <dirname>        Directory to save products to. Default is '{:s}'.

      --verbose          Output additional diagnostics.

{:s})V0G0N",
                  basename(argv0),
                  default_config.outdir,
                  "");
}

// --------------------------------------------------------------- run floor map
//
bool run_floor_map(const ::perceive::pipeline::CliArgs& config) noexcept
{
   using namespace perceive::pipeline;

   MovieResults::Params in_params;

   { // -- (*) -- Initialize params, including loading the scene
      TRACE("begin init-params");
      if(!in_params.init(config, false)) return false;
      TRACE("END   init-params");
   }

   { // -- (*) -- Load the scene
      TRACE("load scene");
      if(!in_params.load_manifest()) return false;
      Expects(in_params.scene_desc);
      INFO(format("Loaded Scene: {:s}\n\n",
                  in_params.scene_desc->scene_info.to_string()));
   }

   // -- (*) -- Create 'MovieResults'
   unique_ptr<MovieResults> movie;
   try {
      TRACE("Create MovieResults");
      movie = make_unique<MovieResults>(in_params);
      TRACE("DONE create MovieResults");
   } catch(std::exception& e) {
      LOG_ERR(format("exception creating MovieResults: {:s}", e.what()));
      return false;
   }
   const auto p = movie->params();

   { // -- (*) -- Save the parameters that we've loaded
      TRACE("Save loaded parameters");
      Json::Value o = in_params.params.to_json();
      Expects(has_key(o, "config") == false);
      o["config"]       = config.to_json();
      o["frame_params"] = movie->frame_results->params_to_json();
      const auto fname  = format("{:s}/params.json", config.outdir);
      file_put_contents(fname, str(o));
   }

   // -- (*) -- Get the movie result
   TRACE("Get movie task results");
   auto movie_ret = movie->movie_task.get_result_synchronized();

   { // -- (*) -- Set the frame number
      const auto frame_no = std::min<unsigned>(4, movie_ret->n_frames() - 1);
      TRACE(format("set frame-no to {}", frame_no));
      movie->frame_results->set_frame(int(frame_no));
   }

   { // -- (*) -- Output 'dot-graph'
      TRACE(format("outputing dot-graph to directory '{:s}'", p.config.outdir));
      movie->frame_results->output_dot_graph(p.config.outdir);
   }

   shared_ptr<const floor_hist::Result> hist_ret;
   { // -- (*) -- Get the floor histogram
      TRACE(format("get the floor histogram"));
      auto& hist_task                    = movie->frame_results->floor_hist;
      auto hist_p                        = hist_task.params();
      hist_p.out_dir                     = config.outdir;
      hist_p.hist_params.color_histogram = true;
      hist_task.set_params(hist_p);
      hist_ret = hist_task.get_result_synchronized();
      if(!hist_ret) return false;
   }

   { // -- (*) -- Save the rectified images
      TRACE(format("save rectified images"));
      const auto& scene_desc = in_params.scene_desc;

      const int n_cams = scene_desc->n_cameras();
      for(auto cam_ind = 0; cam_ind < n_cams; ++cam_ind) {
         const int n_sensors = scene_desc->n_sensors_for(cam_ind);
         for(auto sensor_no = 0; sensor_no < n_sensors; ++sensor_no) {
            const auto sensor_ind
                = scene_desc->sensor_lookup(cam_ind, sensor_no);
            TRACE(format("process {cam={}, {:s}}, {sensor={}, {:s}}",
                         cam_ind,
                         scene_desc->scene_info.bcam_keys[size_t(cam_ind)],
                         sensor_no,
                         scene_desc->sensor_ids[size_t(sensor_ind)]));

            const auto rect_ret = hist_ret->rect_result(cam_ind, sensor_no);
            if(!rect_ret) {
               LOG_ERR(
                   format("failed to load rect-result for camera {}, sensor {}",
                          cam_ind,
                          sensor_no));
               return false;
            }
            const auto fname
                = format("{:s}/rect_{:s}[{}]_{:s}.png",
                         p.config.outdir,
                         scene_desc->scene_info.bcam_keys[size_t(cam_ind)],
                         sensor_no,
                         scene_desc->sensor_ids[size_t(sensor_ind)]);
            cv::imwrite(fname, rect_ret->rectified);
         }
      }
   }

   return true;
}

// -------------------------------------------------------------------- run main
//
int run_main(int argc, char** argv)
{
   Config config;
   auto has_error = false;

   // ---- Parse command line

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      try {
         if(arg == "-h" || arg == "--help") {
            config.show_help = true;
         } else if(arg == "-m"s || arg == "--manifest"s) {
            config.manifest_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--m-json"s) {
            config.manifest_json_str = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-p"s) {
            config.params_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--p-json"s) {
            config.params_json_str = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-d"s) {
            config.outdir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--verbose"s or arg == "-v"s) {
            config.is_verbose = true;
         }
      } catch(std::runtime_error& e) {
         cout << format("Error on command-line: {:s}", e.what()) << endl;
         has_error = true;
      }
   }

   if(has_error) {
      cout << format("aborting...") << endl;
      return EXIT_FAILURE;
   }

   // ---- Process command line

   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(!config.params_fname.empty() and !is_regular_file(config.params_fname)) {
      cout << format("Failed to find parameters file '{:s}'",
                     config.params_fname)
           << endl;
      has_error = true;
   }

   if(!is_directory(config.outdir)) {
      if(is_directory(dirname(config.outdir)))
         mkdir(config.outdir);
      else {
         cout << format("Failed to find output directory '{:s}'", config.outdir)
              << endl;
         has_error = true;
      }
   }

   Json::Value pipeline_params{Json::nullValue};
   if(!config.params_json_str.empty()) {
      if(!parse_json(config.params_json_str, pipeline_params)) {
         LOG_ERR(format("failed to parse params json string: '{:s}'",
                        config.params_json_str));
         has_error = true;
      }
   } else if(!config.params_fname.empty()) {
      try {
         pipeline_params = parse_json(file_get_contents(config.params_fname));
      } catch(std::exception& e) {
         LOG_ERR(format("failed to load json file '{:s}': {:s}",
                        config.params_fname,
                        e.what()));
         has_error = true;
      }
   }

   string manifest_opt_search_path = ""s;
   Json::Value manifest_params{Json::nullValue};
   if(!config.manifest_json_str.empty()) {
      if(!parse_json(config.manifest_json_str, manifest_params)) {
         LOG_ERR(format("failed to parse params json string: '{:s}'",
                        config.manifest_json_str));
         has_error = true;
      }
   } else if(!config.manifest_fname.empty()) {
      try {
         manifest_params = parse_json(file_get_contents(config.manifest_fname));
         manifest_opt_search_path
             = dirname(absolute_path(config.manifest_fname));
      } catch(std::exception& e) {
         LOG_ERR(format("failed to load json file '{:s}': {:s}",
                        config.manifest_fname,
                        e.what()));
         has_error = true;
      }
   } else {
      LOG_ERR(format("Must specify a manifest filename, or inline json data!"));
      has_error = true;
   }

   if(has_error) { return EXIT_FAILURE; }

   load_environment_variables();

   // ---- Action
   pipeline::CliArgs pconf;
   pconf.pipeline_params          = pipeline_params;
   pconf.manifest_params          = manifest_params;
   pconf.manifest_opt_search_path = manifest_opt_search_path;
   pconf.no_stats                 = true;
   pconf.generate_tracks          = false;
   pconf.outdir                   = config.outdir;
   // pconf.n_frames_to_process = 4;
   pconf.save_localization_data = false;
   pconf.on_the_fly_still       = true;
   // pconf.colour_floor_hist   = true;
   if(config.is_verbose) pconf.verbosity = 2;

   const bool success = run_floor_map(pconf);

   if(success) INFO(format("output saved to '{:s}'", config.outdir));

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::render_floor_map
