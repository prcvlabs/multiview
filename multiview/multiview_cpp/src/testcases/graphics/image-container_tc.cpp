
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
CATCH_TEST_CASE("ImageContainerMakePatch", "[image_container_make_patch]")
{
   CATCH_SECTION("image-container-make-patch")
   {
      return;
      const int w = 30;
      const int h = 10;
      auto im     = cv::Mat(h, w, CV_8UC3);
      make_patch(10, 3, Vector2(0.0, 0.0), Vector2(2.0, 0.0), im);
   }
}

} // namespace perceive
