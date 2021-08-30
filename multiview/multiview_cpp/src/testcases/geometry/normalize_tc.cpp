
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/normalize-data.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/math.hpp"

namespace perceive
{
CATCH_TEST_CASE("NormalizeData", "[normalize-data]")
{
   CATCH_SECTION("normalize-data")
   {

      auto test_vec = [&](const vector<Vector3>& Xs) {
         SVD3DRet O = svd_3d_UDV(cbegin(Xs), cend(Xs));
         cout << str(O) << endl;

         const auto q = rot3x3_to_quaternion(O.V.transpose());
         vector<Vector3> Ys;
         Ys.reserve(Xs.size());
         std::transform(cbegin(Xs),
                        cend(Xs),
                        std::back_inserter(Ys),
                        [&](const auto& X) { return q.apply(X); });

         SVD3DRet P = svd_3d_UDV(cbegin(Ys), cend(Ys));
         cout << str(P) << endl;
      };

      if(false) {
         vector<Vector3> Xs(10);
         Xs[0] = Vector3{0.0, 2.0, 1.0};
         Xs[1] = Vector3{0.0, 1.0, 0.0};
         Xs[2] = Vector3{0.0, -1.0, 0.0};
         Xs[3] = Vector3{0.0, -2.0, 0.0};
         Xs[4] = Vector3{1.0, 1.0, 0.0};
         Xs[5] = Vector3{-1.0, -1.0, 0.0};
         Xs[6] = Vector3{-1.0, -1.0, 1.0};
         Xs[7] = Vector3{-10.0, 0.0, -1.0};
         Xs[8] = Vector3{0.5, -2.0, 1.0};
         Xs[9] = Vector3{4.0, 2.0, 1.0};
         test_vec(Xs);
      }
   }
}

} // namespace perceive
