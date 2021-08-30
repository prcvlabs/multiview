
#include "config.hpp"

#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/file-system.hpp"

#define This Config

namespace perceive::gui
{
// ------------------------------------------------------------------- show-help

void show_help(string_view argv0) noexcept
{
   Config config;
   cout << format(R"V0G0N(

   Usage: {}

      -d <dirname>    Output directory. Default is {}.

      -a <dirname>    Source directory for annotation files.
      -p              Print found annotations files, and exit.

)V0G0N",
                  basename(argv0),
                  config.outdir);
}

// ---------------------------------------------------------- parse-command-line

Config parse_command_line(int argc, char** argv) noexcept
{
   Config o;

   { // Search for "-h"/"--help"
      for(int i = 1; i < argc; ++i) {
         string_view arg = argv[i];
         if(arg == "-h" || arg == "--help") o.show_help = true;
      }
      if(o.show_help) return o;
   }

   for(int i = 1; i < argc; ++i) {
      string_view arg = argv[i];
      try {
         if(arg == "-p" || arg == "--print") {
            o.print_files = true;
         } else if(arg == "-y"s) {
            o.allow_overwrite = true;
         } else if(arg == "-d"s) {
            o.outdir = cli::safe_arg_str(argc, argv, i);
         } else if(arg == "-a") {
            o.annotation_dir = cli::safe_arg_str(argc, argv, i);
         } else {
            cout << format("Unknown command-line argument '{}'", arg) << endl;
            o.has_error = true;
         }
      } catch(std::runtime_error& e) {
         cout << format("Error on command-line: {}", e.what()) << endl;
         o.has_error = true;
      }
   }

   // ----------------- fill in more details

   const auto& cache_dir = perceive::multiview_cache_dir();
   o.gui_ini_fname       = format("{}/annotation-app_imgui.ini", cache_dir);

   // ----------------- sanity checks

   if(!is_directory(cache_dir)) { // cache-directory
      cout << format("Failed to find cache-directory '{}'", cache_dir) << endl;
      o.has_error = true;
   }

   if(o.annotation_dir.empty()) {
      cout << format("Must specify an annotation directory!") << endl;
      o.has_error = true;
   } else if(!is_directory(o.annotation_dir)) {
      cout << format("Failed to find annotation directory '{}'.",
                     o.annotation_dir)
           << endl;
      o.has_error = true;
   }

   if(o.outdir.empty()) { // output directory
      cout << format("Must set an output directory!") << endl;
      o.has_error = true;
   } else if(!is_directory(o.outdir)) {
      if(!mkdir_p(o.outdir)) {
         cout << format("Failed to create output directory '{}'", o.outdir)
              << endl;
         o.has_error = true;
      }
   }

   // ----------------- create the manifest
   if(!o.has_error) {
      o.manifest  = make_manifest(o);
      o.has_error = o.manifest.has_error;
   }

   return o;
}

} // namespace perceive::gui

#undef This
