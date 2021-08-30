
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/calibration/aruco-cube.hpp"

namespace perceive
{
CATCH_TEST_CASE("ArucoCube", "[aruco-cube]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("aruco-cube")
   { //
      const auto ac   = make_kyle_aruco_cube();
      const auto ac_s = ac.to_json_str();

      ArucoCube bc;
      read(bc, ac_s);

      const auto bc_s = bc.to_json_str();

      if(false) {
         cout << ac_s << endl;
         cout << endl;
         cout << bc_s << endl;
      }

      CATCH_REQUIRE(ac_s == bc_s);
   }
}

} // namespace perceive
