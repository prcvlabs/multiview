
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "perceive/contrib/catch.hpp"
#include "perceive/optimization/shortest-path.hpp"

namespace perceive
{
// ------------------------------------------------------------- print path info
//
template<typename T>
static string
print_path_infoT(const vector<T>& path,
                 std::function<real(const T& u, const T& v)> edge_weight,
                 std::function<string(const T& u)> to_str) noexcept
{
   std::stringstream ss{""};
   for(auto i = 0u; i < path.size(); ++i) {
      if(i > 0) {
         ss << format("\n          -- {} --> ",
                      edge_weight(path[i - 1], path[i]));
      }
      ss << to_str(path[i]);
   }
   return ss.str();
}

static string print_path_info(
    const vector<Point3>& path,
    std::function<real(const Point3& u, const Point3& v)> edge_weight) noexcept
{
   return print_path_infoT<Point3>(path, edge_weight, [](const Point3& x) {
      return format("[{}, {}, {}]", x.x, x.y, x.z);
   });
}

static string print_path_info(
    const vector<int>& path,
    std::function<real(const int& u, const int& v)> edge_weight) noexcept
{
   return print_path_infoT<int>(
       path, edge_weight, [](const int& x) { return format("[{}]", x); });
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("ShortestPathSparse", "[shortest-path-sparse]")
{
   // std::mt19937 g;
   // g.seed(1);

   // std::uniform_int_distribution<int> distribution{1, 5};
   // std::uniform_int_distribution<int> random_M{-9999, 9999};

   CATCH_SECTION("shortest-path-sparse")
   {
      //

      auto edge_weight = [](const auto& u, const auto& v) -> real {
         return (u - v).quadrance();
      };

      auto for_each_neighbour
          = [](const auto& u, std::function<void(const Point3&)> f) {
               if(u.z < 10) {
                  for(auto dy = -1; dy <= 1; ++dy)
                     for(auto dx = -1; dx <= 1; ++dx) {
                        Point3 a{u.x + dx, u.y + dy, u.z + 1};
                        if(a.x >= 0 and a.y >= 0 and a.x <= 9 and a.y <= 9) {
                           f(a);
                        }
                     }
               }
            };

      {
         const auto path = shortest_path_sparse<Point3>(
             {0, 0, 0}, {9, 9, 9}, edge_weight, for_each_neighbour);

         CATCH_REQUIRE(path.size() == 10);
         CATCH_REQUIRE(path.front() == Point3{0, 0, 0});
         CATCH_REQUIRE(path.back() == Point3{9, 9, 9});
      }

      {
         const auto path = shortest_path_sparse<Point3>(
             {0, 0, 0}, {5, 5, 3}, edge_weight, for_each_neighbour);
         CATCH_REQUIRE(path.size() == 0);
      }

      {
         const auto path = shortest_path_sparse<Point3>(
             {0, 0, 0}, {0, 0, 5}, edge_weight, for_each_neighbour);
         CATCH_REQUIRE(path.size() == 6);
      }
   }
}

} // namespace perceive
