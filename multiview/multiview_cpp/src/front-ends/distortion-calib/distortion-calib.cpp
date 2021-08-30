
#include "distortion-calib-inc.hpp"
#include "stdinc.hpp"

#include "perceive/foundation.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#include "perceive/calibration/run-distort-calibration.hpp"

// Models
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"

namespace perceive::distortion_calib
{
// ---------------------------------------------------------------------- brief

string brief() noexcept { return "Distortion calibration."; }

// ---------------------------------------------------------------------- config

struct Config
{
   bool show_help{false};
   bool allow_overwrite{false};
   bool fast_calib{true};
   int nx{15};
   int ny{10};
   real square_size{0.0508};
   string sensor_id{""};
   string manifest_file{""};
   string out_file{""};
   string out_dir{"/tmp"};
   string type{""};
};

// ------------------------------------------------------------------- show help

void show_help(string argv0)
{
   Config conf;
   const string k_light_grey{"\x1b[37m"s};
   const string k_clear{"\x1b[0m"s};

   cout << format(R"V0G0N(

Usage: {:s} [OPTIONS...] <manifest-filename>

   Performs distortion calibration (which replaces the intrinsic matrix) for a 
   given manifest file. The manifest file is a text file that lists an image 
   filename, and, optionally, the measured distance between camera and grid. 
   (Lines starting with '#' are ignored.)

Options:   

   {:s}# Output options{:s}
   -o <filename>            Output filename.
   -y                       Allow overwrite of output filename.
   -d <directory>           Output directory, for saving temporary files useful
                            for diagnositics. Default is '{:s}'.

   {:s}# Configuration options{:s}
   -s <sensor-id>           Must be something like 'STR00001'
   --nx <integer>           Calibration pattern cols. Default is '{:s}'.
   --ny <integer>           Calibration pattern rows. Default is '{:s}'.
   --square-size <float>    Size of one square on an image. Default is {:s}.
   --calc-distort-inverse   Recalculate the distortion inverse.

{:s})V0G0N",
                  basename(argv0),
                  k_light_grey,
                  k_clear,
                  conf.out_dir,
                  k_light_grey,
                  k_clear,
                  conf.nx,
                  conf.ny,
                  conf.square_size,
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
         } else if(arg == "-o"s) {
            config.out_file = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-d"s) {
            config.out_dir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-y"s) {
            config.allow_overwrite = true;
         } else if(arg == "-s"s) {
            config.sensor_id = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--nx"s) {
            config.nx = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--ny"s) {
            config.ny = cli::safe_arg_int(argc, argv, i);
         } else if(arg == "--square-size"s) {
            config.square_size = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--fast"s) {
            config.fast_calib = true;
         } else if(arg == "--accurate"s) {
            config.fast_calib = false;
         } else if(i + 1 == argc) {
            config.manifest_file = arg;
         } else {
            cout << format("Unexpected command-line argument: '{:s}'", arg)
                 << endl;
            has_error = true;
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

   if(config.manifest_file == "") {
      cout << format("Must specify a manifest file, aborting") << endl;
      return EXIT_FAILURE;
   }

   if(config.out_file == "") {
      cout << format("Must specify an output file, aborting") << endl;
      return EXIT_FAILURE;
   }

   if(!is_s3_uri(config.manifest_file)
      && !is_regular_file(config.manifest_file)) {
      cout << format("Failed to find manifest file '{:s}'",
                     config.manifest_file)
           << endl;
      return EXIT_FAILURE;
   }

   if(!is_directory(config.out_dir)) {
      cout << format("Failed to find directory '{:s}'", config.out_dir) << endl;
      return EXIT_FAILURE;
   }

   if(config.sensor_id == "") {
      cout << format("Must specify a sensor-id") << endl;
      return EXIT_FAILURE;
   }

   if(!config.allow_overwrite && is_regular_file(config.out_file)) {
      cout << format("Cowardly refusing to overwrite file '{:s}'",
                     config.out_file)
           << endl;
      return EXIT_FAILURE;
   }

   // ---- Action
   using calibration::run_distort_calibration;
   auto success = run_distort_calibration(unsigned(config.nx),
                                          unsigned(config.ny),
                                          config.square_size,
                                          config.sensor_id,
                                          config.manifest_file,
                                          config.fast_calib,
                                          config.out_file,
                                          config.out_dir);
   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::distortion_calib
