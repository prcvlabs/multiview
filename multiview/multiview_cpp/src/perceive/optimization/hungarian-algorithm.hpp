
#pragma once

#include "perceive/utils/concepts.hpp"

namespace perceive
{
namespace detail
{
   vector<std::pair<unsigned, unsigned>> hungarian_algorithm(
       const unsigned n_lhs_verts,
       const unsigned n_rhs_verts,
       std::function<double(unsigned l, unsigned r)> cost) noexcept;
}

//
// Minimum cost bipartite graph matching.
//
// 'n-lhs-verts' on the left hand side
// 'n-rhs-verts' on the right hand side
// and 'cost' gives the cost for matching an left-vertex with a right-vertex.
//
// Returns a set of pairs that is the optimal matching.
// O(n^2)
//
template<std::integral I>
inline vector<std::pair<unsigned, unsigned>>
hungarian_algorithm(const I n_lhs_verts,
                    const I n_rhs_verts,
                    std::function<double(unsigned l, unsigned r)> cost) noexcept
{
   static constexpr uint64_t u_max = std::numeric_limits<unsigned>::max();

   if constexpr(std::is_signed<I>::value) {
      Expects(n_lhs_verts >= 0);
      Expects(n_rhs_verts >= 0);
   }
   Expects(uint64_t(n_lhs_verts) <= u_max);
   Expects(uint64_t(n_rhs_verts) <= u_max);

   return detail::hungarian_algorithm(
       unsigned(n_lhs_verts), unsigned(n_rhs_verts), cost);
}

} // namespace perceive
