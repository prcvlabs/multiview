
#include "stdinc.hpp"

#include "plane-set-calib-inc.hpp"

#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

#include "perceive/calibration/plane-set/run-calibration.hpp"

namespace perceive::plane_set_calib
{
// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "Not sure, something to do with planes and 3D fitting.";
}

// ---------------------------------------------------------------------- config

struct Config
{
   bool show_help{false}; //
   bool allow_overwrite{false};
   string out_dir{"/tmp"};
   string scene_fname{""};
   int bcam_index{-1};
   string calib_plane_set_fname{""};
   string complete_cps_fname{""};
   string params_fname{""};
   string image_fname;
};

// ------------------------------------------------------------------- show help

void show_help(string argv0)
{
   cout << format(R"V0G0N(

   Usage: {:s} [OPTIONS...] <image-filename>

      --scene <filename>         Path to scene file.
      --bcam-index <integer>     Which bcam to optimize (in scene).
      --cps  <filename>          Path to CalibPlaneSet filename.
      --complete-cps <filename>  For creating disparity map.

      -y                         Allow overwrite of output file
    
      -d <dirname>               Directory to save to.  

      --params <filename>        Params filename


{:s})V0G0N",
                  basename(argv0),
                  "");
}

// -------------------------------------------------------------------- run main

int run_main(int argc, char** argv)
{
   Config config;
   auto has_error = false;

   // ---- Parse command line

   for(int i = 1; i < argc; ++i) {
      string arg = argv[i];
      try {
         if(arg == "-h" || arg == "--help") {
            config.show_help = true;
         } else if(arg == "-y") {
            config.allow_overwrite = true;
         } else if(arg == "-d") {
            config.out_dir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--scene") {
            config.scene_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--bcam-index") {
            config.bcam_index = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--cps") {
            config.calib_plane_set_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--complete-cps") {
            config.complete_cps_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--params") {
            config.params_fname = cli::safe_arg_str(argc, argv, i);
         } else {
            if(!config.image_fname.empty()) {
               throw std::runtime_error(
                   format("attempt to set image file to "
                          "'{:s}'; however, it was previously "
                          "set to '{:s}'",
                          arg,
                          config.image_fname));
            }

            config.image_fname = arg;
         }

      } catch(std::runtime_error& e) {
         cout << format("error on command-line: {:s}", e.what()) << endl;
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

   if(config.scene_fname == "") {
      cout << format("must specify a scene file") << endl;
      has_error = true;
   }

   if(!is_directory(config.out_dir)) {
      cout << format("failed to find output directory '{:s}'", config.out_dir)
           << endl;
      has_error = true;
   }

   if(!is_regular_file(config.scene_fname)) {
      cout << format("file not found: '{:s}'", config.scene_fname) << endl;
      has_error = true;
   }

   if(config.bcam_index < 0) {
      cout << format("must specify a (postive) bcam-index") << endl;
      has_error = true;
   }

   if(!is_regular_file(config.calib_plane_set_fname)) {
      cout << format("file not found: '{:s}'", config.calib_plane_set_fname)
           << endl;
      has_error = true;
   }

   if(config.complete_cps_fname != ""
      and !is_regular_file(config.complete_cps_fname)) {
      cout << format("file not found: '{:s}'", config.complete_cps_fname)
           << endl;
      has_error = true;
   }

   if(!is_regular_file(config.image_fname)) {
      cout << format("file not found: '{:s}'", config.image_fname) << endl;
      has_error = true;
   }

   if(has_error) { return EXIT_FAILURE; }

   // ---- Params setup
   calibration::PlaneSetCalibParams params;
   params.scene_fname           = config.scene_fname;
   params.bcam_index            = config.bcam_index;
   params.image_fname           = config.image_fname;
   params.calib_plane_set_fname = config.calib_plane_set_fname;
   params.complete_cps_fname    = config.complete_cps_fname;
   params.params_fname          = config.params_fname;
   params.outdir                = config.out_dir;

   // ---- Action
   bool success = calibration::run_plane_set_calibration(params);

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::plane_set_calib
