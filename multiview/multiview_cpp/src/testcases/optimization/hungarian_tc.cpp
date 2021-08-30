
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "perceive/contrib/catch.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"

namespace perceive
{
static real
assignment_cost(std::function<double(unsigned l, unsigned r)> cost_func,
                const vector<std::pair<unsigned, unsigned>>& matching) noexcept
{
   real cost = 0.0;
   for(const auto& m : matching) cost += cost_func(m.first, m.second);
   return cost;
};

// ------------------------------------------------------------- print path info
//
static string
print_path_info(const unsigned n_lhs_verts,
                const unsigned n_rhs_verts,
                std::function<double(unsigned l, unsigned r)> cost_func,
                const vector<std::pair<unsigned, unsigned>>& matching) noexcept
{
   std::stringstream ss("");

   auto is_matching = [&](const unsigned r, const unsigned c) {
      const auto ii
          = std::find_if(begin(matching), end(matching), [&](const auto& x) {
               return (x.first == r and x.second == c);
            });
      return ii != end(matching);
   };

   ss << format("|L|  = {}", n_lhs_verts) << endl;
   ss << format("|R|  = {}", n_rhs_verts) << endl;
   ss << format("cost = {}", assignment_cost(cost_func, matching)) << endl;
   for(auto r = 0u; r < n_lhs_verts; ++r) {
      for(auto c = 0u; c < n_rhs_verts; ++c) {
         if(c > 0) ss << "  ";
         const bool it_is = is_matching(r, c);
         if(it_is) ss << ANSI_COLOUR_BLUE_BG;
         ss << format("{:5.3f}", cost_func(r, c));
         if(it_is) ss << ANSI_COLOUR_RESET;
      }
      ss << endl;
   }

   return ss.str();
}

// ------------------------------------------------------------- naive hungarian
//
static vector<std::pair<unsigned, unsigned>> naive_hungarian(
    const unsigned n_lhs_verts,
    const unsigned n_rhs_verts,
    std::function<double(unsigned l, unsigned r)> cost_func) noexcept
{
   //
   const bool is_l_assigned = n_lhs_verts >= n_rhs_verts;
   const unsigned sz        = std::max(n_lhs_verts, n_rhs_verts);
   vector<unsigned> assignments(sz);
   std::iota(begin(assignments), end(assignments), 0);

   auto make_assignment = [&](const vector<unsigned>& assignments) {
      vector<std::pair<unsigned, unsigned>> o;
      o.reserve(sz);
      if(is_l_assigned) {
         for(auto i = 0u; i < sz; ++i)
            if(assignments[i] < n_rhs_verts) o.push_back({i, assignments[i]});
      } else {
         for(auto i = 0u; i < sz; ++i)
            if(assignments[i] < n_lhs_verts) o.push_back({assignments[i], i});
      }
      return o;
   };

   auto best      = make_assignment(assignments);
   auto best_cost = assignment_cost(cost_func, best);

   while(std::next_permutation(begin(assignments), end(assignments))) {
      const auto matching = make_assignment(assignments);
      const auto cost     = assignment_cost(cost_func, matching);
      if(cost < best_cost) {
         best      = matching;
         best_cost = cost;
      }
   }

   return best;
}

// --------------------------------------------------------- test-test hungarian
//
static void test_test_hungarian()
{
   auto test_M = [&](const MatrixXr& M) {
      auto f = [&](unsigned r, unsigned c) { return M(r, c); };
      cout << "M = \n" << M << endl << endl;
      const auto matching0
          = hungarian_algorithm(unsigned(M.rows()), unsigned(M.cols()), f);
      const auto matching1
          = naive_hungarian(unsigned(M.rows()), unsigned(M.cols()), f);
      cout << "Hungarian produced:" << endl;
      cout << print_path_info(
          unsigned(M.rows()), unsigned(M.cols()), f, matching0)
           << endl;
      cout << endl << "Naive produced:" << endl;
      cout << print_path_info(
          unsigned(M.rows()), unsigned(M.cols()), f, matching1)
           << endl;
   };

   MatrixXr M(3, 3);
   M << 40, 60, 15, 25, 30, 45, 55, 30, 25;
   test_M(M);

   M << 30, 25, 10, 15, 10, 20, 25, 20, 15;
   test_M(M);

   {
      MatrixXr M(5, 5);
      M << 8.463, 4.191, 3.132, 0.0, 0.0, 6.852, 5.245, 2.044, 0.0, 0.0, 4.434,
          8.781, 2.295, 0.0, 0.0, 0.273, 5.344, 6.704, 0.0, 0.0, 9.139, 4.173,
          4.572, 0.0, 0.0;
      test_M(M);
   }
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("HungarianAlgorithm", "[hungarian-algorithm]")
{
   std::mt19937 g;
   g.seed(1);

   std::uniform_int_distribution<int> distribution{1, 5};
   std::uniform_int_distribution<int> random_M{-9999, 9999};

   CATCH_SECTION("hungarian-algorithm")
   {
      auto test_it = [&](const bool is_degenerate) {
         const auto n_rows = distribution(g);
         const auto n_cols = distribution(g);
         MatrixXr M(n_rows, n_cols);
         for(auto r = 0; r < n_rows; ++r)
            for(auto c = 0; c < n_cols; ++c)
               M(r, c) = is_degenerate
                             ? real(distribution(g) % 2 + 1)
                             : real(random_M(g)) / 1000.0; // [0..9.99]

         auto f = [&](unsigned r, unsigned c) { return M(r, c); };
         const auto matching0
             = hungarian_algorithm(unsigned(M.rows()), unsigned(M.cols()), f);
         const auto matching1
             = naive_hungarian(unsigned(M.rows()), unsigned(M.cols()), f);
         const auto cost0 = assignment_cost(f, matching0);
         const auto cost1 = assignment_cost(f, matching1);
         if(!is_close(cost0, cost1)) {
            LOG_ERR(format("Error in hungarian algorithm output:"));
            cout << "Naive Hungarian produced:" << endl;
            cout << print_path_info(
                unsigned(M.rows()), unsigned(M.cols()), f, matching1)
                 << endl;
            cout << endl << "Hungarian produced:" << endl;
            cout << print_path_info(
                unsigned(M.rows()), unsigned(M.cols()), f, matching0)
                 << endl;
            CATCH_REQUIRE(false);
         } else {
            CATCH_REQUIRE(true);
         }
      };

      for(auto i = 0; i < 1000; ++i) test_it(true); // test degenerate cases
      for(auto i = 0; i < 1000; ++i) test_it(false);
      // test_test_hungarian();
   }
}

} // namespace perceive
