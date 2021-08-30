
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"
#include "stdinc.hpp"
#include "undistort-inc.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive::undistort
{
// ---------------------------------------------------------------------- config
//
struct Config
{
   bool show_help                    = false;
   bool has_error                    = false;
   string sensor_id                  = ""s;
   string in_file                    = ""s;
   string out_file                   = ""s;
   bool force_recalc_cache_undistort = false;
   bool allow_overwrite              = false;

   string to_string() const noexcept
   {
      return format(R"V0G0N(
   show-help:        {:s}
   has-error:        {:s}
   sensor-id:       '{:s}'
   in-file:         '{:s}'
   out-file:        '{:s}'
   force-recalc:     {:s}
   allow-overwrite:  {:s}
)V0G0N",
                    str(show_help),
                    str(has_error),
                    sensor_id,
                    in_file,
                    out_file,
                    str(force_recalc_cache_undistort),
                    str(allow_overwrite));
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

   Usage: {:s} [OPTIONS...] <out-filename>

      -s <sensor-id>  Image sensor.
      -i <filename>   Input filename.
      -f              Force recalculationg of cache undistort file.
      -y              Allow overwrite of output file.

   Example:

      > {:s} -s STR00009 -i distorted-image.png undistorted-image.png

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
   vector<string> outfiles;
   for(int i{1}; i < argc; ++i) {
      string arg = argv[i];
      try {
         if(arg == "-s"s) {
            config.sensor_id = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-i"s) {
            config.in_file = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-f"s) {
            config.force_recalc_cache_undistort = true;
         } else if(arg == "-y"s) {
            config.allow_overwrite = true;
         } else {
            outfiles.push_back(arg);
            config.out_file = arg;
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

   if(config.in_file.empty()) {
      cout << format("Must specify an input filename.\n");
      config.has_error = true;
   } else if(!is_regular_file(config.in_file)) {
      cout << format("Could not find input image file '{:s}'.\n", config.in_file);
      config.has_error = true;
   }

   if(outfiles.size() == 0) {
      cout << format("Must specify an output filename.\n");
      config.has_error = true;
   } else if(outfiles.size() > 1) {
      cout << format("Output file specified multiple times; saw: ['{:s}'].\n",
                     implode(cbegin(outfiles), cend(outfiles), "', '"));
      config.has_error = true;
   } else if(is_regular_file(config.out_file) and !config.allow_overwrite) {
      cout << format("Cowardly refusing to overwrite output file '{:s}'.\n",
                     config.out_file);
      config.has_error = true;
   } else if(config.out_file.empty()) {
      cout << format("Output file was not specified!\n"); // It could happen
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

   // ---- Load the image
   auto load_image = [](const string& filename) {
      cv::Mat im = cv::imread(filename, cv::IMREAD_UNCHANGED);
      if(im.empty()) FATAL(format("failed to load image '{:s}'", filename));
      INFO(format("loaded '{:s}'", basename(filename)));
      return im;
   };
   const auto im = load_image(config.in_file);

   // ---- Load the sensor distortion model
   DistortionModel M;
   fetch(M, config.sensor_id);
   CachingUndistortInverse cu(M, config.force_recalc_cache_undistort);

   // ---- Undistort
   cu.set_working_format(im.cols, im.rows);
   const auto w = 800; // im.rows * 2.0;
   const auto h = 600; // im.rows;

   cv::Mat mapx, mapy;

   const auto focal_length = 0.5 * real(h) / tan(0.5 * to_radians(80.0));
   Matrix3r H              = Matrix3r::Identity();
   Matrix3r K              = Matrix3r::Identity();
   K(0, 0) = K(1, 1) = focal_length;
   K(0, 2)           = 0.5 * w;
   K(1, 2)           = 0.5 * h;

   ParallelJobSet pjobs;
   auto f = [&](const Vector2& x) { return cu.distort(x); };
   create_cv_remap_threaded(w, h, H, f, K, mapx, mapy, pjobs);

   cv::Mat undist;
   cv::remap(im,
             undist,
             mapx,
             mapy,
             cv::INTER_LINEAR,
             cv::BORDER_CONSTANT,
             cv::Scalar(255, 255, 255));

   cv::imwrite(config.out_file, undist);

   return EXIT_SUCCESS;
}

} // namespace perceive::undistort
