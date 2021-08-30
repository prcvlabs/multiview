
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "bcam_file.hpp"

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/io/perceive-assets.hpp"

namespace perceive
{
CATCH_TEST_CASE("PolynomialModel", "[polynomial_model]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("PolynomialModelConstruction")
   {
      DistortionModel m0, m1, m2;
      m1 = m0;
      m2 = std::move(m0);
      CATCH_REQUIRE(true);
   }

   // Test CachedUndistort
   CATCH_SECTION("PolynomialModelCachedUndistort")
   {
      // if(false) { // skipping this *long* testcase
      //    DistortionModel m;
      //    fetch(m, "STR00049", DataSource::MUTLVIEW_DATA);
      //    CachingUndistortInverse cu(
      //        m, true, AABBi{0, 0, 0, 0}, false, 60.f, true);
      //    cu.set_working_format(m.calib_format().x, m.calib_format().y);

      //    // Compare error from 'cu' and 'm'
      //    std::deque<real> errs;
      //    for(auto x = -1.0; x <= 1.0; x += 0.1) {
      //       for(auto y = -1.0; y <= 1.0; y += 0.1) {
      //          auto D1 = m.distort(Vector2(x, y));
      //          auto D2 = cu.distort(Vector2(x, y));

      //          if(D2.is_finite()) errs.emplace_back((D1 - D2).norm());
      //       }
      //    }

      //    auto stats = calc_sample_statistics(begin(errs), end(errs));
      //    cout << "ZAPPY" << endl;
      //    cout << str(stats) << endl;
      //    CATCH_REQUIRE(stats.N > 0);
      //    CATCH_REQUIRE(std::isfinite(stats.average));
      // }
   }

   CATCH_SECTION("caching-undistort-test")
   {
      if(false) {
         BinocularCameraInfo bcam_info;
         read(bcam_info, test::C0001024_v2_str());
         const auto& M = bcam_info.M[0]; // the sensor to test

         INFO(format("Read bcam, testing sensor {:s}", M.sensor_id()));

         // Load the caching-distort-inverse
         const CachingUndistortInverse cu{M};
      }
   }
}

} // namespace perceive
