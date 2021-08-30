
#include "stdinc.hpp"

#include "cache-undistort-regen-inc.hpp"

#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive::cache_undistort_regen
{
// ---------------------------------------------------------------------- config
//
struct Config
{
   bool show_help                    = false;
   bool has_error                    = false;
   string sensor_id                  = ""s;
   bool force_recalc_cache_undistort = false;

   string to_string() const noexcept
   {
      return format(R"V0G0N(
   show-help:        {:s}
   has-error:        {:s}
   sensor-id:       '{:s}'
   force-recalc:     {:s}
)V0G0N",
                    str(show_help),
                    str(has_error),
                    sensor_id,
                    str(force_recalc_cache_undistort));
   }

   friend string str(const Config& config) noexcept
   {
      return config.to_string();
   }
};

// ------------------------------------------------------------------- Show Help
//
static void show_help(string arg0)
{
   auto exec = basename(arg0);
   cout << format(R"V0G0N(

   Usage: {:s} [-f] <sensor-id>

   Example:

      > {:s} -f STR00009

)V0G0N",
                  exec,
                  exec);
}

// ---------------------------------------------------------- parse command line
//
static Config parse_command_line(int argc, char** argv)
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
         if(arg == "-f"s) {
            config.force_recalc_cache_undistort = true;
         } else {
            if(!config.sensor_id.empty()) {
               throw std::runtime_error(
                   format("sensor-id was set to '{}' but then a later attempt "
                          "to set it to '{}'",
                          config.sensor_id,
                          arg));
            }
            config.sensor_id = arg;
         }
      } catch(std::runtime_error& e) {
         cout << format("Error on command-line: {:s}", e.what()) << endl;
         config.has_error = true;
      }
   }
   if(config.has_error) return config;

   // -- Sanity checks
   if(config.sensor_id.empty()) {
      cout << format("Must specify a sensor id.\n");
      config.has_error = true;
   }

   return config;
}

// -------------------------------------------------------------------- Run Main
//
int run_main(int argc, char** argv)
{
   cout << format("Welcome to 'undistort'") << endl;

   const auto config = parse_command_line(argc, argv);
   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }
   if(config.has_error) {
      INFO("printing configuration before aborting:");
      cout << str(config) << endl;
      return EXIT_FAILURE;
   }

   load_environment_variables();
   cout << environment_info() << config.to_string() << endl;

   const auto sensor_id = config.sensor_id;

   // ---- Load the sensor distortion model
   DistortionModel M;
   fetch(M, config.sensor_id);
   CachingUndistortInverse cu(M, config.force_recalc_cache_undistort);

   return EXIT_SUCCESS;
}

} // namespace perceive::cache_undistort_regen
