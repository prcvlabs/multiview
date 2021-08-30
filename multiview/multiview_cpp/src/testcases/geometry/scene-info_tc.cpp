
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/scene/scene-description-info.hpp"

namespace perceive
{
CATCH_TEST_CASE("Read/write scene-info", "[read_write_scene_info]")
{
   CATCH_SECTION("read-write-scenes")
   {
      SceneDescriptionInfo sd_info1, sd_info2;

      for(auto i = 0; i < 10; ++i) {
         sd_info1.cad_model_key = format("sprite-{:8d}", i);

         string out1, out2;
         write(sd_info1, out1);
         read(sd_info2, out1);
         write(sd_info2, out2);

         CATCH_REQUIRE(sd_info1 == sd_info2);

         sd_info1.bcam_keys.push_back(format("bcam-key-{:8d}", i));
         sd_info1.bcam_transforms.emplace_back(Vector3(i, i, i),
                                               Quaternion(1, 0, 0, 1.0));
      }
   }
}

} // namespace perceive
