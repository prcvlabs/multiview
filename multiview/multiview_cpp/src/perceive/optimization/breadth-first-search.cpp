
#include "breadth-first-search.hpp"

#include <deque>

namespace perceive
{
std::vector<unsigned> breadth_first_search(
    const unsigned source,
    std::function<bool(const unsigned u)> is_target,
    const std::size_t n_vertices,
    std::function<void(const unsigned u, std::function<void(const unsigned)>)>
        for_each_neighbour) noexcept
{
   Expects(for_each_neighbour);
   Expects(size_t(source) < n_vertices);

   if(is_target(source)) return {source};

   std::vector<int> parent_v(n_vertices);
   std::fill(begin(parent_v), end(parent_v), -1);

   auto is_labeled = [&](unsigned v) { return parent_v[v] >= 0; };

   int sink = -1;
   std::deque<unsigned> Q; // a queue
   Q.push_back(source);
   while(Q.size() > 0 and sink == -1) {
      const auto v = Q.front();
      Q.pop_front();
      for_each_neighbour(v, [&](const unsigned w) {
         Expects(size_t(w) < n_vertices);
         if(!is_labeled(w)) {
            parent_v[w] = int(v);
            if(is_target(w))
               sink = int(w);
            else
               Q.push_back(w);
         }
      });
   }

   if(sink == -1) return {};

   vector<unsigned> out;
   {
      Q.clear();
      Q.push_back(unsigned(sink));
      while(Q.back() != source) Q.push_back(unsigned(parent_v[Q.back()]));
      out.reserve(Q.size());
      std::copy(rbegin(Q), rend(Q), std::back_inserter(out));
   }

   return out;
}

std::vector<unsigned> breadth_first_search(
    const unsigned source,
    const unsigned sink,
    const std::size_t n_vertices,
    std::function<void(const unsigned u, std::function<void(const unsigned)>)>
        for_each_neighbour) noexcept
{
   return breadth_first_search(
       source,
       [sink](const auto u) { return u == sink; },
       n_vertices,
       for_each_neighbour);
}

} // namespace perceive
