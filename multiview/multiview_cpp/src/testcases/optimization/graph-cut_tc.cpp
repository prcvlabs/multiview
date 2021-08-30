
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "perceive/contrib/catch.hpp"
#include "perceive/optimization/min-cut-graph.hpp"

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
CATCH_TEST_CASE("GraphCut", "[graph-cut]")
{
   CATCH_SECTION("graph-cut")
   {
      MinCutGraph g;
      g.set_n_vertices(5);

      unsigned source_id = 3;
      unsigned sink_id   = 4;

      g.add_edge(source_id, 0, 2);
      g.add_edge(source_id, 1, 1);
      g.add_edge(source_id, 2, 2);

      g.add_edge(0, 1, 0.5);
      g.add_edge(1, 2, 0.5);

      g.add_edge(sink_id, 0, 1);
      g.add_edge(sink_id, 1, 2);
      g.add_edge(sink_id, 2, 1);

      const auto cut = g.min_cut(source_id, sink_id);

      auto vertex_label_function = [&](unsigned ind) {
         if(ind == source_id) return "\u03b1"s;
         if(ind == sink_id) return "\u03b2"s;
         return format("{:c}", char('A' + ind));
      };
      auto edge_label_function
          = [&](unsigned ind0, unsigned ind1) { return format(""); };

      g.save_graphviz("/tmp/graph-cut.dot",
                      vertex_label_function,
                      edge_label_function,
                      &cut);

      CATCH_REQUIRE(std::fabs(cut.flow - 4.0) < 1e-4);

      // INFO("HELLO GRAPH CUT");
      // INFO(format("flow was {}", cut.flow));
   }
}

} // namespace perceive
