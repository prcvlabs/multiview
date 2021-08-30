
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/calibration/plane-set/calib-plane-set.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/colour-set.hpp"

namespace perceive
{
using namespace perceive::calibration;

// -----------------------------------------------------------------------------

CATCH_TEST_CASE("CalibPlaneSet", "[calib_plane_set]")
{
   //
   // ------------------------------------------------ calib-plane-set_load-save
   //
   CATCH_SECTION("calib-plane-set_load-save")
   {
      auto make_calib_plane = [&]() {
         auto cp       = CalibPlane{};
         cp.p3         = Plane(1, 0, 0, 4);
         cp.plane_type = 0;

         cp.l_spixel_indices.resize(4);
         std::iota(begin(cp.l_spixel_indices), end(cp.l_spixel_indices), 1);
         cp.r_spixel_indices.resize(4);
         std::iota(begin(cp.r_spixel_indices), end(cp.r_spixel_indices), 4);

         return cp;
      };

      auto test_read_write = [&](const CalibPlaneSet& cps) {
         CalibPlaneSet cp2;
         string s;
         write(cps, s);
         read(cp2, s);
         CATCH_REQUIRE(cps == cp2);
      };

      {
         CalibPlaneSet empty_cps;
         test_read_write(empty_cps);
      }

      CalibPlaneSet cps2;
      cps2.p3s.push_back(make_calib_plane());
      test_read_write(cps2);

      cps2.p3s.push_back(make_calib_plane());
      test_read_write(cps2);
   }
}

} // namespace perceive
