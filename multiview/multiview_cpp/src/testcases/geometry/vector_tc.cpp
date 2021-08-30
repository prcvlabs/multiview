
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
CATCH_TEST_CASE("QuantizeVec2", "[quantize_vec2]")
{
   CATCH_SECTION("quantize-vec2")
   {
      return;
      auto test_it = [&](const Vector2& X) {
         const auto qspace = quantize(X);
         INFO(format("X = {}", str(X)));
         for(const auto& [x, weight] : qspace) {
            cout << format("   [{}, {}]  -->  {}", x.x, x.y, weight) << endl;
         }
         cout << endl;
      };

      for(auto i = -1.0; i <= 1.0; i += 0.2)
         for(auto j = -1.0; j <= 1.0; j += 0.2) test_it(Vector2(i, j));
   }
}

} // namespace perceive
