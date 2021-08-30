
#include "position-scene-cameras-inc.hpp"

#include "perceive/calibration/position-scene-cameras/config.hpp"
#include "perceive/calibration/position-scene-cameras/helpers.hpp"
#include "perceive/calibration/position-scene-cameras/manifest-data.hpp"
#include "perceive/calibration/position-scene-cameras/print-helper.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::position_scene_cameras
{
using namespace perceive::calibration::position_scene_cameras;

// -------------------------------------------------------------------- run-main
//
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

   if(!is_directory(config.outdir)) {
      if(!mkdir_p(config.outdir)) {
         cout << format("Failed to create output directory: '{:s}'",
                        config.outdir)
              << endl;
         return EXIT_FAILURE;
      }
   }

   bool has_error   = false;
   auto parse_mdata = [&]() {
      ManifestData mdata;
      try {
         mdata = parse_manifest_file(config);
      } catch(std::exception& e) {
         has_error = true;
      }
      return mdata;
   };

   const string_view outdir = config.outdir;
   print(g_info, g_default, format("output directory set to '{}'", outdir));

   // Turn the input file in `ManifestData`
   const ManifestData mdata = parse_mdata();
   if(has_error) return EXIT_FAILURE;

   // Perform optimization
   const auto [ets, success] = optimize_n_way(config.scene_id, mdata, outdir);

   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::position_scene_cameras
