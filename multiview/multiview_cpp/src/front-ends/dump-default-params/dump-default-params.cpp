
#include "stdinc.hpp"

#include "dump-default-params-inc.hpp"

#include "perceive/pipeline/frame-results.hpp"
#include "perceive/utils/cli-utils.hpp"

namespace perceive::dump_default_params
{
// ---------------------------------------------------------------------- brief

string brief() noexcept { return "Dumps the default 'params.json' to file."; }

// ---------------------------------------------------------------------- config

struct Config
{
   bool has_error        = false;
   bool show_help        = false;
   string outdir         = "/tmp"s;
   string manifest_fname = ""s;
};

// -------------------------------------------------------------------- run main

void show_help(string argv0)
{
   Config default_config;

   cout << format(R"V0G0N(

   Usage: {:s} [OPTIONS...] <manifest-filename>

      -d <dirname>       Directory to save products to. Default is '{:s}'.

   Example:
 
      # 
      > {:s} -d /tmp/params /path/to/manifest/file

{:s})V0G0N",
                  basename(argv0),
                  default_config.outdir,
                  basename(argv0),
                  "");
}

int run_main(int argc, char** argv)
{
   Config config;
   auto has_error = false;

   // ---- Parse command line
   for(int i = 1; i < argc; ++i) {
      const string_view arg = argv[i];
      if(arg == "-h" || arg == "--help") config.show_help = true;
   }

   if(!config.show_help) {
      for(int i = 1; i < argc - 1; ++i) {
         const string_view arg = argv[i];
         try {
            if(arg == "-h" || arg == "--help") {
               config.show_help = true;
            } else if(arg == "-d"s) {
               config.outdir = cli::safe_arg_str(argc - 1, argv, i);
            }
         } catch(std::runtime_error& e) {
            cout << format("Error on command-line: {:s}", e.what()) << endl;
            has_error = true;
         }
      }

      if(argc == 1) {
         cout << format("Error: must specify a <scene>") << endl;
         has_error = true;
      } else {
         config.manifest_fname = argv[argc - 1];
      }
   }

   if(config.manifest_fname == ""s) {
      cout << format("Must specify a manifest filename!") << endl;
      has_error = true;
   } else if(!is_regular_file(config.manifest_fname)) {
      cout << format("Failed to find manifest file: '{:s}'",
                     config.manifest_fname)
           << endl;
      has_error = true;
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

   if(!is_directory(config.outdir)) {
      if(is_directory(dirname(config.outdir)))
         mkdir(config.outdir);
      else {
         cout << format("Failed to find output directory '{:s}'", config.outdir)
              << endl;
         has_error = true;
      }
   }

   if(has_error) { return EXIT_FAILURE; }

   SceneDescription::InitOptions opts;
   opts.optional_search_path = dirname(config.manifest_fname);
   opts.on_the_fly_still     = true;
   auto scene_desc = load_scene_from_manifest(config.manifest_fname, opts);
   if(scene_desc == nullptr) {
      LOG_ERR(format("failed to load manifest '{:s}', aborting",
                     config.manifest_fname));
      return EXIT_FAILURE;
   }

   // ---- Action
   bool success = false;
   try {
      pipeline::FrameResults frame_results{scene_desc, true};

      const string fname = format("{:s}/default-params.json", config.outdir);
      Json::Value o      = Json::Value{Json::objectValue};
      o["frame_params"]  = frame_results.params_to_json();
      file_put_contents(fname, str(o));
      INFO(format("output parameters saved to '{:s}'", fname));
      success = true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed: {:s}", e.what()));
   }

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::dump_default_params
