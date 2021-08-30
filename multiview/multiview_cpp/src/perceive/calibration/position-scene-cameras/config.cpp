
#include "config.hpp"

#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::calibration::position_scene_cameras
{
// ------------------------------------------------------------------- show-help

void show_help(string argv0)
{
   Config config;

   cout << format(R"V0G0N(

   Usage: {:s} Options.

   Options:

      -m <filename>         Manifest filename.
      --aruco-cube <key>    The cube to use. Default is the bespoke "kyle-cube".
      --aruco-file <fname>  Json filename to load cube data from.
      --scene-id <name>     Mandatory.

      -d <dirname>          Output Directory. Default is '{:s}'.

   Manifest:

      The manifest file is a plain text file, with each line specifying
      an input, as follows:

         ```
         # Lines beginning with '#' are ignored.
         # Position  Camera   Image Filename
         Aruco1* C0001028_v3  images/aruco1/stereo-c1028-2019-10-15T21:02:01.jpg
         Aruco1  C0001035_v3  images/aruco1/stereo-c1035-2019-10-15T21:01:58.jpg
         Aruco1  C0001036_v5  images/aruco1/stereo-c1036-2019-10-15T21:02:04.jpg
         Aruco2  C0001026_v4  images/aruco2/stereo-c1026-2019-10-15T21:08:17.jpg
         Aruco2  C0001028_v3  images/aruco2/stereo-c1028-2019-10-15T21:08:14.jpg
         Aruco3  C0001026_v4  images/aruco3/stereo-c1026-2019-10-15T21:14:32.jpg
         Aruco3  C0001058_v3  images/aruco3/stereo-c1058-2019-10-15T21:14:29.jpg
         Aruco4  C0001026_v4  images/aruco4/stereo-c1026-2019-10-15T21:24:08.jpg
         Aruco4  C0001031_v3  images/aruco4/stereo-c1031-2019-10-15T21:24:11.jpg
         Aruco5  C0001027_v3  images/aruco5/stereo-c1027-2019-10-15T21:26:12.jpg
         Aruco5  C0001031_v3  images/aruco5/stereo-c1031-2019-10-15T21:26:15.jpg
         Aruco6  C0001027_v3  images/aruco6/stereo-c1027-2019-10-15T21:28:35.jpg
         Aruco6  C0001030_v4  images/aruco6/stereo-c1030-2019-10-15T21:28:14.jpg
         Aruco6  C0001030_v4  images/aruco6/stereo-c1030-2019-10-15T21:28:38.jpg
         Aruco7  C0001027_v3  images/aruco7/stereo-c1027-2019-10-15T21:29:58.jpg
         Aruco7  C0001034_v6  images/aruco7/stereo-c1034-2019-10-15T21:30:01.jpg
         ```

      Note [1]: Image filenames are relative to the manifest.text file location
      Note [2]: At least 1 position must have a '*' somewhere. This position
                sets the coordinate system of the scene.

   TODO:

      * Automagically load up a scene, and estimate the bounding plane.

{:s})V0G0N",
                  basename(argv0),
                  config.outdir,
                  "");
}

// ---------------------------------------------------------- parse-command-line
//
Config parse_command_line(int argc, char** argv) noexcept
{
   Config config;

   // -- Look for -h/--help
   for(int i{1}; i < argc; ++i)
      if(strcmp(argv[i], "-h") == 0 or strcmp(argv[i], "--help") == 0)
         config.show_help = true;
   if(config.show_help) return config;

   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      try {
         if(arg == "-d") {
            config.outdir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--scene-id") {
            config.scene_id = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-m") {
            config.manifest_fname = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "--aruco-cube"s) {
            config.cube_key = cli::safe_arg_str(argc, argv, i);
         } else {
            cout << format("Unexpected command-line argument: {:s}\n", arg);
            config.has_error = true;
         }
      } catch(std::exception& e) {
         cout << format("Error on command-line: {:s}\n", e.what());
         config.has_error = true;
      }
   }

   // -- Sanity checks
   // Did we specify a scene-id ?
   if(config.scene_id.empty()) {
      cout << format("Error, must specify the scene-id! (i.e., '--scene-id "
                     "<scene-id>')\n");
      config.has_error = true;
   } else if(std::find_if(cbegin(config.scene_id),
                          cend(config.scene_id),
                          [](auto c) {
                             return !(std::isalnum(c) or c == '_' or c == '-');
                          })
             != cend(config.scene_id)) {
      cout << format("Error, cannot have special characters in a scene-id!\n");
      config.has_error = true;
   }

   if(config.manifest_fname.empty()) {
      cout << format("Error, must specify the manifest filename! (i.e., '-m "
                     "<filename>')\n");
      config.has_error = true;
   }

   if(!config.cube_key.empty() and !config.cube_filename.empty()) {
      cout << format(
          "Error, cannot specify both an acruco-key and an aruco-filename!\n");
      config.has_error = true;
   }

   return config;
}

} // namespace perceive::calibration::position_scene_cameras
