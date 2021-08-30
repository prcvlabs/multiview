
#include "blur-faces-inc.hpp"
#include "stdinc.hpp"

#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/movie/ffmpeg.hpp"
#include "perceive/movie/movie-utils.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace perceive::blur_faces
{
// ----------------------------------------------------------------------- brief

string brief() noexcept { return "Blur faces on an input video"; }

// ---------------------------------------------------------------------- config
//
struct Config
{
   bool has_error       = false;
   bool show_help       = false;
   bool is_verbose      = false;
   bool allow_overwrite = false;
   real frame_rate      = 15.0; // read it from video
   bool render_pose     = false;
   string movie_fname   = ""s;
   string output_fname  = "/tmp/movie.mp4"s;
   string params_fname  = ""s;
};

// ------------------------------------------------------------------- show help
//
void show_help(const string_view argv0)
{
   Config default_config;

   cout << format(R"V0G0N(

   Usage: {:s} [OPTIONS...]

      -i <filename>       Movie file to process.
      -o <filename>       Output movie filename. Default is '{:s}'.
      -y                  Allow overwrite.
      -r <number>         Framerate. Default is '{}'.
      --render-pose       Render poses (skeletons)
      -p <json-file>      Json parameters file. If file does not
                          exist, then parameters are written to the
                          file.

{:s})V0G0N",
                  basename(argv0),
                  default_config.frame_rate,
                  default_config.output_fname,
                  "");
}

// ------------------------------------------------------------------ parse-args

Config parse_args(int argc, char** argv)
{
   Config config;

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      if(arg == "-h" || arg == "--help") config.show_help = true;
   }
   if(config.show_help) return config;

   // ---- Parse command line

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      try {
         if(arg == "-i"s) {
            config.movie_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-o"s) {
            config.output_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-y"s) {
            config.allow_overwrite = true;
         } else if(arg == "-r"s) {
            config.frame_rate = cli::safe_arg_real(argc, argv, i);
         } else if(arg == "--render-pose"s) {
            config.render_pose = true;
         } else if(arg == "--verbose"s or arg == "-v"s) {
            config.is_verbose = true;
         } else if(arg == "-p"s) {
            config.params_fname = cli::safe_arg_str(argc, argv, i);
         } else {
            LOG_ERR(format("unknown command line argument: '{:s}'", arg));
            config.has_error = true;
         }
      } catch(std::runtime_error& e) {
         LOG_ERR(format("Error on command-line: {:s}", e.what()));
         config.has_error = true;
      }
   }

   if(!is_regular_file(config.movie_fname)) {
      LOG_ERR(format("failed to find file: '{:s}'", config.movie_fname));
      config.has_error = true;
   }

   if(config.output_fname.empty()) {
      LOG_ERR(format("failed to specify an output filename"));
      config.has_error = true;
   }

   if(!config.output_fname.empty() && !config.allow_overwrite
      && is_regular_file(config.output_fname)) {
      LOG_ERR(format("cowardly refusing to overwrite output file '{:s}'",
                     config.output_fname));
      config.has_error = true;
   }

   if(file_ext(config.output_fname) != ".mp4"s) {
      LOG_ERR(format("output file '{:s}' must have extension '.mp4'",
                     config.output_fname));
      config.has_error = true;
   }

   if(config.frame_rate <= 0.0) {
      LOG_ERR(
          format("cowardly refusing create output video with frame-rate '{}'",
                 config.frame_rate));
      config.has_error = true;
   }

   return config;
}

// -------------------------------------------------------------------- run-main

int run_main(int argc, char** argv)
{
   // ---- Handle configuration
   const Config config = parse_args(argc, argv);

   if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.has_error) {
      cout << "aborting" << endl;
      return EXIT_FAILURE;
   }

   // Handle Openpose parameters
   pose_skeleton::Params op_params;
   if(!config.params_fname.empty()) {
      if(is_regular_file(config.params_fname)) {
         try { // load
            load(op_params, config.params_fname);
         } catch(std::runtime_error& e) {
            LOG_ERR(format("failed to load parameters from '{:s}': {:s}",
                           config.params_fname,
                           e.what()));
            return EXIT_FAILURE;
         }
      } else {
         try { // save
            save(op_params, config.params_fname);
         } catch(std::runtime_error& e) {
            LOG_ERR(format("failed to save parameters to '{:s}': {:s}",
                           config.params_fname,
                           e.what()));
            return EXIT_FAILURE;
         }
      }
   }

   // Try to open the movie
   auto video = make_unique<cv::VideoCapture>(config.movie_fname);
   if(!video->isOpened())
      FATAL(format("failed to open movie file '{:s}'", config.movie_fname));

   // Try to get a frame-rate
   auto get_fps = [&config](const auto& video) {
      if(config.frame_rate > 0.0) return config.frame_rate;
      const auto fps = video->get(cv::CAP_PROP_FPS);
      if(fps == 0.0) {
         WARN(format("failed to query FPS from video file"));
         return 15.0;
      }
      return fps;
   };
   const auto fps = get_fps(video);
   INFO(format("using {} FPS", fps));

   // Get pose detection ready
   PoseSkeletonExec op_exec;

   // Grab the first frame, so that we can query the frame width and height
   cv::Mat im;
   *video >> im;
   if(im.empty())
      FATAL(format("failed to read a single frame from video file, aborting"));
   const int width  = im.cols;
   const int height = im.rows;

   auto encoder = movie::StreamingFFMpegEncoder::create(
       config.output_fname, width, height, config.frame_rate);

   { // ---- Process the movie
      int frame_no = 0;
      while(true) {
         // blur the faces
         auto pose_ret = op_exec.run(im, op_params, "/tmp");
         movie::blur_faces(im, frame_no, config.render_pose, pose_ret.pose(0));

         // Backup save
         cv::imwrite(format("/tmp/{:4d}.png", frame_no + 1), im);

         // encode the frame
         // encoder.push_frame(im);
         *video >> im;
         ++frame_no;
         if(im.empty()) break;
      }
   }
   run_exit_functions();

   return encoder.close();
}

} // namespace perceive::blur_faces
