
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "perceive/contrib/catch.hpp"
#include "perceive/optimization/breadth-first-search.hpp"

namespace perceive
{
// ------------------------------------------------------------- print path info
//
static string path_str(const vector<unsigned>& path) noexcept
{
   return format("[{:s}]", implode(cbegin(path), cend(path), "] => ["));
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("BreadthFirstSearch", "[breadth-first-search]")
{
   CATCH_SECTION("breadth-first-search")
   {
      vector<vector<unsigned>> edges
          = {{{1, 2}, {0, 3, 4}, {1}, {1, 4}, {1, 3}, {6}, {5}}};

      const auto N = edges.size();
      auto fe_neighbour
          = [&](const unsigned u, std::function<void(const unsigned)> f) {
               Expects(u < N);
               std::for_each(cbegin(edges[u]), cend(edges[u]), f);
            };

      {
         const auto path = breadth_first_search(0, 3, N, fe_neighbour);
         // cout << path_str(path) << endl;
         CATCH_REQUIRE(path.size() == 3);
      }

      {
         const auto path = breadth_first_search(0, 0, N, fe_neighbour);
         // cout << path_str(path) << endl;
         CATCH_REQUIRE(path.size() == 1);
      }

      {
         const auto path = breadth_first_search(0, 5, N, fe_neighbour);
         // cout << path_str(path) << endl;
         CATCH_REQUIRE(path.size() == 0);
      }

      {
         const auto path = breadth_first_search(6, 5, N, fe_neighbour);
         // cout << path_str(path) << endl;
         CATCH_REQUIRE(path.size() == 2);
      }
   }
}

} // namespace perceive
