
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/scene/scene-description.hpp"
#include "perceive/utils/file-system.hpp"
#include "testcases/testcase-data.hpp"

namespace perceive
{
// ----------------------------------------------------------- testcase-manifest
// The manifest of testcase 007, if it has been fetched
// (build/fetch-testcases.sh 7).

static string testcase_manifest_filename() noexcept
{
   return format("{}/testcases/007_site03_sparse_tracks/manifest.json",
                 test::testcase_data_dir());
}

// -------------------------------------------------------------------- testcase
// Hidden: initialising a scene decodes a frame of every camera, and loads
// (or, when they are not cached, regenerates for minutes) the undistort
// maps of every sensor. Run it by name: multiview-testcases "[.still]"

CATCH_TEST_CASE("StillAccumulate", "[.still]")
{
   const auto fname = testcase_manifest_filename();
   if(!is_regular_file(fname)) {
      WARN(format("testcase manifest '{}' not found, skipping", fname));
      return;
   }
   const Json::Value manifest_data = parse_json(file_get_contents(fname));

   auto still_after_one_frame = [&](const bool accumulate_still) {
      SceneDescription scene_desc;
      SceneDescription::InitOptions opts;
      opts.optional_search_path = dirname(absolute_path(fname));
      opts.no_stereo            = true;
      opts.accumulate_still     = accumulate_still;
      scene_desc.init_from_json(manifest_data, opts);
      const auto images = scene_desc.get_images_for_frame(1);
      CATCH_REQUIRE(images.frame_loaded);
      const auto& im = images.sensor_images.at(0);
      return std::make_tuple(scene_desc.accum_blurred_LAB(0),
                             unsigned(im.cols),
                             unsigned(im.rows));
   };

   CATCH_SECTION("accumulate-still")
   {
      const auto [still, w, h] = still_after_one_frame(true);
      CATCH_REQUIRE(still.width == w);
      CATCH_REQUIRE(still.height == h);
   }

   CATCH_SECTION("no-still")
   {
      const auto [still, w, h] = still_after_one_frame(false);
      CATCH_REQUIRE(w > 0);
      CATCH_REQUIRE(still.size() == 0);
   }
}

} // namespace perceive
