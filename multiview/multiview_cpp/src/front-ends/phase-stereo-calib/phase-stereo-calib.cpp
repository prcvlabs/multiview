
#include "stdinc.hpp"

#include "phase-stereo-calib-inc.hpp"

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/calibration/phase-position.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::phase_stereo_calib
{
// ----------------------------------------------------------------------- brief

string brief() noexcept { return "Method for stereo calibration."; }

// ---------------------------------------------------------------------- config

struct Config
{
   bool show_help{false};
   bool has_error{false};
   vector<string> filenames{}; // input filenames
   string camera_id{""s};      // id of camera, sets output filename
   array<string, 2> sensor_names{{""s, ""s}};
   real baseline_est{0.14854};
   bool use_fast_distort{false};
   bool render_cube_only{false};
   bool animate_cube{false};
   string cube_key{""s};
   string cube_filename{""s};
   bool K_opt{false};
   string outdir{"/tmp"s};
};

// ------------------------------------------------------------------- show help

void show_help(string argv0)
{
   Config config;

   cout << format(R"V0G0N(

   Usage: {:s} [OPTIONS...] <image-filenames>*

      -d  <dirname>        Output directory. Default is '{:s}'.

      -s0 <sensor-name>    Left sensor name.  Something like 'STR00042'.
      -s1 <sensor-name>    Right sensor name. Something like 'STR00041'.
      --cam-id <camera-id> Id of the output camera. Something like 'C000042_v1'

      --baseline <number>  Measured distance between camera sensors. Default {}.

      --fast-distort        Faster distort method that clips edges of images.
      --full-distort        Distorted/Undistort the full image.

      --aruco-cube <key>   The cube to use. Default is the bespoke "kyle-cube".
      --aruco-file <fname> Json filename to load cube data from.

      --render-cube-only   Do not perform calibration, but instead, only render
                           the loaded aruco cube.
      --animate-cube       Produce a cube animation

      --K-opt              Experimental K optimization procedure.
 
{:s})V0G0N",
                  basename(argv0),
                  config.outdir,
                  config.baseline_est,
                  "");
}

// ---------------------------------------------------------- parse command line

Config parse_command_line(int argc, char** argv)
{
   Config config;

   // -- Look for -h/--help
   for(int i{1}; i < argc; ++i)
      if(strcmp(argv[i], "-h") == 0 or strcmp(argv[i], "--help") == 0)
         config.show_help = true;
   if(config.show_help) return config;

   // --
   for(int i{1}; i < argc; ++i) {
      string arg = argv[i];
      try {
         if(arg == "-d"s) {
            config.outdir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-s0"s) {
            config.sensor_names[0] = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-s1"s) {
            config.sensor_names[1] = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--cam-id"s) {
            config.camera_id = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--baseline"s) {
            config.baseline_est = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--fast-distort"s) {
            config.use_fast_distort = true;
         } else if(arg == "--full-distort"s) {
            config.use_fast_distort = false;
         } else if(arg == "--render-cube-only"s) {
            config.render_cube_only = true;
         } else if(arg == "--animate-cube"s) {
            config.animate_cube = true;
         } else if(arg == "--aruco-cube"s) {
            config.cube_key = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--aruco-file"s) {
            config.cube_filename = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--K-opt"s) {
            config.K_opt = true;
         } else {
            config.filenames.push_back(arg);
         }
      } catch(std::runtime_error& e) {
         cout << format("Error on command-line: {:s}", e.what()) << endl;
         config.has_error = true;
      }
   }
   if(config.has_error) return config;

   // -- Sanity checks
   if(!config.render_cube_only and config.filenames.size() < 1) {
      cout << format("Error, must specify at least one input filename\n");
      config.has_error = true;
   }

   for(const auto& filename : config.filenames) {
      INFO(format("got filename {:s}", filename));
      if(!is_s3_uri(filename) && !is_regular_file(filename)) {
         cout << format("Error, filename not found: '{:s}'\n", filename);
         config.has_error = true;
      }
   }

   if(!config.render_cube_only and config.sensor_names[0].empty()) {
      cout << format(
          "Error, must specify a sensor 0 (i.e., '-s0 <sensor-name>')\n");
      config.has_error = true;
   }

   if(!config.render_cube_only and config.sensor_names[1].empty()) {
      cout << format(
          "Error, must specify a sensor 1 (i.e., '-s1 <sensor-name>')\n");
      config.has_error = true;
   }

   if(!config.render_cube_only and config.camera_id.empty()) {
      cout << format(
          "Error, must specify a camera id! (i.e., '--cam-id <camera-id>')\n");
      config.has_error = true;
   }

   if(!config.cube_filename.empty() and !is_s3_uri(config.cube_filename)
      and !is_regular_file(config.cube_filename)) {
      cout << format("Error, failed to find aruco-file '{:s}'\n",
                     config.cube_filename);
      config.has_error = true;
   }

   return config;
}

// -------------------------------------------------------------------- run main

int run_main(int argc, char** argv)
{
   const auto config = parse_command_line(argc, argv);
   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      cout << "Aborting because of previous errors..." << endl;
      return EXIT_FAILURE;
   }

   load_environment_variables();
   INFO(perceive::format("Multiview Configuration:"));
   std::cout << perceive::environment_info() << std::endl;

   calibration::PhasePositionParams p;
   if(config.cube_key.empty() and config.cube_filename.empty()) {
      p.ac = make_kyle_aruco_cube();
   } else if(!config.cube_key.empty()) {
      fetch(p.ac, config.cube_key);
   } else if(!config.cube_filename.empty()) {
      string raw_data;
      lazy_load(config.cube_filename, raw_data);
      read(p.ac, raw_data);
   } else {
      FATAL("logic error");
   }

   p.camera_id          = config.camera_id;
   p.input_image_fnames = config.filenames;
   try {
      p.raw_input_images.resize(config.filenames.size());
      for(auto j = 0u; j < config.filenames.size(); ++j) {
         vector<char> raw_data;
         lazy_load(config.filenames[j], raw_data);
         p.raw_input_images[j] = decode_image(raw_data);
         // ARGBImage::load(config.filenames[j]);
      }
      if(!config.render_cube_only)
         for(size_t i = 0; i < 2; ++i)
            fetch(p.bcam_info.M[i], config.sensor_names[i]);
   } catch(std::runtime_error& e) {
      cout << e.what() << endl;
      return EXIT_FAILURE;
   }
   p.images.resize(p.raw_input_images.size());
   for(auto j = 0u; j < p.raw_input_images.size(); ++j)
      hsplit(p.raw_input_images[j], p.images[j][0], p.images[j][1]);
   p.outdir               = config.outdir;
   p.baseline_estimate    = config.baseline_est;
   p.K_opt                = config.K_opt;
   p.estimate_bcam_qt     = true;
   p.use_pointwise_result = true;
   p.use_fast_distort     = config.use_fast_distort;
   p.render_cube_only     = config.render_cube_only;
   p.animate_cube         = config.animate_cube;

   const bool success = run_phase_position_calibration(p);

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::phase_stereo_calib
