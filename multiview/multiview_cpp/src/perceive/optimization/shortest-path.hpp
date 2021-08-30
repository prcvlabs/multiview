
#pragma once

// #include <boost/container/pmr/monotonic_buffer_resource.hpp>
// #include <boost/container/pmr/set.hpp>
// #include <boost/container/pmr/unsynchronized_pool_resource.hpp>
// #include <experimental/memory_resource>
#include <set>

namespace perceive
{
//
// Shortest path on a DAG, using sparse representations
//
template<typename V>
inline vector<V> shortest_path_sparse(
    const V source,
    const V sink,
    std::function<real(const V& u, const V& v)> edge_weight,
    std::function<void(const V& u, std::function<void(const V&)>)>
        for_each_neighbour,
    const real k_max_cost = std::numeric_limits<real>::max())
{
   if(source == sink) return {};

   Expects(edge_weight);
   Expects(for_each_neighbour);

   using Key     = std::pair<real, V>;
   using Compare = std::less<Key>;

   // boost::container::pmr::monotonic_buffer_resource alloc(1024 * 10);
   std::set<Key, Compare> Q;

   std::unordered_map<V, V> parent_v;
   std::unordered_map<V, real> dist_;

   real max_cost = std::isfinite(k_max_cost) ? k_max_cost
                                             : std::numeric_limits<real>::max();

   auto dist = [&](const V u) {
      auto ii = dist_.find(u);
      return (ii == cend(dist_)) ? max_cost : ii->second;
   };

   Q.insert({0.0, source});
   dist_[source]   = 0.0;
   real path_score = dNAN;

   while(Q.size() > 0) {
      const V u = Q.begin()->second;
      Q.erase(Q.begin()); // Remove element
      if(u == sink) {
         Expects(parent_v.find(u) != cend(parent_v));
         break; // We're done
      }
      Expects(dist(u) < std::numeric_limits<real>::max());
      const auto dist_u = dist(u);

      for_each_neighbour(u, [&](const V v) {
         const real w = edge_weight(u, v);
         Expects(std::isfinite(w));
         const auto dist_v = dist(v);
         if(dist_v > dist_u + w) {
            Q.erase({dist_v, v});
            const auto new_dist_v = dist_u + w;
            Q.insert({new_dist_v, v});
            parent_v[v] = u;
            dist_[v]    = new_dist_v;
         }
      });
   }

   if(parent_v.find(sink) == cend(parent_v)) return {};

   // Reconstruct the path
   vector<V> out;
   {
      out.reserve(Q.size() + 2);
      out.push_back(sink);
      while(out.back() != source) {
         const auto ii = parent_v.find(out.back());
         Expects(ii != cend(parent_v));
         out.push_back(ii->second);
      }
      std::reverse(begin(out), end(out));
   }

   return out;
}

} // namespace perceive
