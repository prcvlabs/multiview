
#include "min-cut-graph.hpp"

//#include "perceive/foundation.hpp"
#define INFO(m) printf("%s:%d  %s\n", __FILE__, int(__LINE__), m);

#include <stdint.h>

// #include <unordered_set>
#include <algorithm> // for std::for_each
#include <exception>
#include <unordered_map>
#include <unordered_set>
#include <utility> // for std::pair

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/graph/topological_sort.hpp>
// #include <boost/graph/read_dimacs.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>

#define This MinCutGraph

using std::cout;
using std::endl;
using std::string;
using std::swap;

// -------------------------------------------------------------------- typedefs
struct VertexProperties
{
   bool alpha_expansion_vertex{false};
   // boost::vertex_color_t z{boost::default_color_type};
   // double boost::vertex_distance_t;
};

typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>
    FTraits;

typedef boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::directedS,

    boost::property<
        boost::vertex_index_t,
        unsigned,
        boost::property<
            boost::vertex_color_t,
            boost::default_color_type,
            boost::property<boost::vertex_distance_t,
                            double,
                            boost::property<boost::vertex_predecessor_t,
                                            FTraits::edge_descriptor>>>>,

    boost::property<
        boost::edge_weight_t,
        float,
        boost::property<
            boost::edge_capacity_t,
            double,
            boost::property<boost::edge_residual_capacity_t,
                            double,
                            boost::property<boost::edge_reverse_t,
                                            FTraits::edge_descriptor>>>>>

    G;

using vertex_descriptor = boost::graph_traits<G>::vertex_descriptor;
using edge_descriptor   = boost::graph_traits<G>::edge_descriptor;
// using weight_map_t      = boost::property_map<G, boost::edge_weight_t>::type;
// using weight_t          = boost::property_traits<weight_map_t>::value_type;
using vertex_iterator = boost::graph_traits<G>::vertex_iterator;
using edge_iterator   = boost::graph_traits<G>::edge_iterator;

namespace perceive
{
// ----------------------------------------------------------------------- Pimpl

static uint64_t key(uint32_t a, uint32_t b) noexcept
{
   return (uint64_t(a) << 32) | (uint64_t(b) << 0);
}

class This::Pimpl
{
 public:
   G g;

   Pimpl(unsigned n_verts)
       : g(n_verts)
   {}

   Pimpl(const Pimpl& rhs) = default;
   Pimpl(Pimpl&& rhs)      = default;
   Pimpl& operator=(const Pimpl& rhs) = default;
   Pimpl& operator=(Pimpl&& rhs) = default;
};

// ---------------------------------------------------------------- Construction

This::MinCutGraph(unsigned n_verts)
    : pimpl_(std::make_unique<Pimpl>(n_verts))
{}

This::MinCutGraph(MinCutGraph&& o) noexcept = default;

This::MinCutGraph(const MinCutGraph& rhs)
    : pimpl_(std::make_unique<Pimpl>(*rhs.pimpl_))
{}

This::~MinCutGraph() = default;

MinCutGraph& This::operator=(MinCutGraph&& o) noexcept = default;

MinCutGraph& This::operator=(const MinCutGraph& rhs)
{
   *pimpl_ = *rhs.pimpl_;
   return *this;
}

// ----------------------------------------------------------- removing vertices

template<typename T>
static void
rm_vertex_helper(T& g, vertex_descriptor v1, vector<uint64_t>& to_remove_buffer)
{
   boost::graph_traits<G>::out_edge_iterator ei, e_end;
   std::tie(ei, e_end) = boost::out_edges(v1, g);
   auto& to_remove     = to_remove_buffer;
   to_remove.clear();
   to_remove.reserve(size_t(std::distance(ei, e_end)));
   for(; ei != e_end; ++ei) {
      uint32_t uu = uint32_t(boost::source(*ei, g));
      uint32_t vv = uint32_t(boost::target(*ei, g));
      to_remove.push_back(key(uu, vv));
      to_remove.push_back(key(vv, uu));
   }

   for(auto uv : to_remove) {
      uint32_t uu = (uv & 0xffffffff00000000ull >> 32);
      uint32_t vv = (uv & 0x00000000ffffffffull >> 0);
      boost::remove_edge(uu, vv, g);
   }

   boost::remove_vertex(v1, g);
}

// ------------------------------------------------------------------ n-vertices

void This::reserve_vertices(unsigned n_verts)
{
   pimpl_->g.m_vertices.reserve(n_verts);
}

void This::reserve_edges(unsigned vert_ind, unsigned n_edges)
{
   auto& g = pimpl_->g;
   if(vert_ind < g.m_vertices.size())
      g.m_vertices[vert_ind].m_out_edges.reserve(n_edges);
}

void This::set_n_vertices(unsigned n_verts)
{
   if(n_verts == n_vertices()) return;

   auto& g = pimpl_->g;
   vector<uint64_t> to_remove_buffer;

   while(n_verts < n_vertices()) {
      rm_vertex_helper(g, boost::vertex(n_vertices() - 1, g), to_remove_buffer);
   }

   g.m_vertices.reserve(n_verts);

   auto counter = n_vertices();
   while(n_vertices() < n_verts) boost::add_vertex(g);

   assert(n_verts == n_vertices());
}

unsigned This::n_vertices() const noexcept { return size(); }
unsigned This::n_edges() const noexcept
{
   return unsigned(boost::num_edges(pimpl_->g));
}
unsigned This::size() const noexcept
{
   return unsigned(boost::num_vertices(pimpl_->g));
}

// ----------------------------------------------------------------------- edges

bool This::has_edge(unsigned v1, unsigned v2) const noexcept
{
   assert(v1 < n_vertices());
   const auto& o = pimpl_->g.m_vertices[v1].m_out_edges;
   const auto ii = std::find(cbegin(o), cend(o), v2);
   return ii != cend(o);
}

bool This::add_edge(unsigned v1, unsigned v2, double weight)
{
   auto& g = pimpl_->g;

   const auto& eweight           = get(boost::edge_weight, g);
   const auto& capacity          = get(boost::edge_capacity, g);
   const auto& residual_capacity = get(boost::edge_residual_capacity, g);
   const auto& reverse_edge      = get(boost::edge_reverse, g);
   const auto& vindex            = get(boost::vertex_index, g);

   edge_descriptor e1, e2;
   bool success = false;

   const auto& vert1       = vindex[v1];
   const auto& vert2       = vindex[v2];
   boost::tie(e1, success) = boost::add_edge(vert1, vert2, g);
   if(!success) return false;
   boost::tie(e2, success) = boost::add_edge(vert2, vert1, g);
   if(!success) {
      boost::remove_edge(vert1, vert2, g);
      return false;
   }

   eweight[e1] = eweight[e2] = float(weight);
   capacity[e1] = capacity[e2] = weight;
   residual_capacity[e1] = residual_capacity[e2] = 0.0;
   reverse_edge[e1]                              = e2;
   reverse_edge[e2]                              = e1;

   return success;
}

void This::remove_edge(unsigned v1, unsigned v2)
{
   boost::remove_edge(v1, v2, pimpl_->g);
   boost::remove_edge(v2, v1, pimpl_->g);
}

void This::remove_all_edges()
{
   auto& g      = pimpl_->g;
   const auto N = n_vertices();

   vector<uint64_t> to_remove;
   for(auto i = 0u; i < N; ++i) {
      boost::graph_traits<G>::out_edge_iterator ei, e_end;
      std::tie(ei, e_end) = boost::out_edges(i, g);
      to_remove.reserve(size_t(std::distance(ei, e_end)));
      for(; ei != e_end; ++ei) {
         uint32_t uu = uint32_t(boost::source(*ei, g));
         uint32_t vv = uint32_t(boost::target(*ei, g));
         to_remove.push_back(key(uu, vv));
         to_remove.push_back(key(vv, uu));
      }
      for(auto uv : to_remove) {
         uint32_t uu = (uv & 0xffffffff00000000ull >> 32);
         uint32_t vv = (uv & 0x00000000ffffffffull >> 0);
         boost::remove_edge(uu, vv, g);
      }
   }
}

// ---------------------------------------------------------------- edge weights

// float This::weight(unsigned v1, unsigned v2) const noexcept
// {
//    auto& g             = pimpl_->g;
//    const auto& eweight = get(boost::edge_weight, g);

//    boost::graph_traits<G>::out_edge_iterator ei, e_end;
//    std::tie(ei, e_end) = out_edges(v1, g);
//    auto ii             = std::find_if(ei, e_end, [&](const auto& o) -> bool {
//       return boost::source(o, g) == v2;
//    });
//    if(ii == e_end) return std::numeric_limits<double>::quiet_NaN();

//    return eweight[*ii];
// }

// bool This::set_weight(unsigned v1, unsigned v2, double weight) noexcept
// {
//    auto& g             = pimpl_->g;
//    auto& e             = pimpl_->edge_idx;
//    const auto& weights = get(boost::edge_weight, g);
//    boost::graph_traits<G>::out_edge_iterator ei, e_end;
//    tie(ei, e_end) = out_edges(v1, g);
//    if(std::distance(ei, e_end) > 20) {
//       auto ee = e.find(key(v1, v2));
//       if(ee == e.end()) return false;
//       weights[ee->second] = weight;
//       return true;
//    }

//    auto ee = boost::edge(v1, v2, g);
//    if(ee.second) weights[ee.first] = weight;
//    return ee.second;
// }

// ----------------------------------------------------- Prepare Alpha Expansion
// using label_of_vertex_func = std::function<unsigned (unsigned)>;
// using vertex_label_cost_func = std::function<double (unsigned, unsigned)>;
// using label_label_cost_func = std::function<double (unsigned, unsigned)>;
// struct AlphaExpansionData {
//     unsigned source_vertex{0};
//     unsigned sink_vertex{0};
//     unsigned n_additional_vertices{0}; // number of verts added
// };
// This::AlphaExpansionData
// This::modify_for_alpha_expansion(unsigned alpha_label,
//                                  label_of_vertex_func   l,
//                                  vertex_label_cost_func f,
//                                  label_label_cost_func  e,
//                                  double smoothing_lambda)
// {
//     return modify_for_alpha_expansion(alpha_label, l, f,
//                                       [&] (unsigned v1, unsigned v2,
//                                            unsigned l1, unsigned l2) {
//                                           return e(l1, l2);
//                                       },
//                                       smoothing_lambda);
// }

// This::AlphaExpansionData
// This::modify_for_alpha_expansion(unsigned alpha_label,
//                                  label_of_vertex_func l,
//                                  vertex_label_cost_func f,
//                                  label_label_cost_func e,
//                                  double smoothing_lambda)
// {
//    AlphaExpansionData dat;
// dat.alpha_label = alpha_label;
// auto& g         = pimpl_->g;
// auto& eidx      = pimpl_->edge_idx;

// const auto N        = size();
// const auto max_cost = 1e9;
// const auto lambda   = smoothing_lambda;

// const auto& weights  = get(boost::edge_weight, g);
// const auto& weights2 = get(boost::edge_weight2, g);

// // dat.source_vertex = n_vertices();
// // dat.sink_vertex = dat.source_vertex + 1;

// // Add the source and sink vertices
// boost::add_vertex(g);
// dat.source_vertex                           = N;
// g[dat.source_vertex].alpha_expansion_vertex = true;
// boost::add_vertex(g);
// dat.sink_vertex                           = N + 1;
// g[dat.sink_vertex].alpha_expansion_vertex = true;

// // Add t-links
// for(auto n = 0u; n < N; ++n) {
//    auto label = l(n);
//    add_edge(n, dat.source_vertex, f(n, alpha_label));
//    add_edge(
//        n, dat.sink_vertex, (label == alpha_label) ? max_cost : f(n,
//        label));
// }

// // Add e-links
// vector<std::pair<unsigned, unsigned>> to_remove;

// for(auto n = 0u; n < N; ++n) {
//    boost::graph_traits<G>::out_edge_iterator ei, e_end;
//    for(tie(ei, e_end) = out_edges(n, g); ei != e_end; ++ei) {
//       auto v1 = boost::source(*ei, g);
//       auto v2 = boost::target(*ei, g);
//       if(v1 >= N || v2 >= N) continue; // don't worry about t-links
//       if(v1 < v2) { // undirected graph... avoid double-counting
//          auto label1 = l(v1);
//          auto label2 = l(v2);
//          if(label1 == label2) {
//             auto ee = boost::edge(v1, v2, g);
//             assert(ee.second);
//             weights2[ee.first] = weights[ee.first];
//             weights[ee.first]  = lambda * e(label1, alpha_label);
//          } else {
//             // labels are different... insert node
//             to_remove.emplace_back(v1, v2);
//          }
//       }
//    }
// }

// // Add the additional nodes
// unsigned new_vertex = N + 2;
// for(const auto& x : to_remove) {
//    auto v1 = x.first;
//    auto v2 = x.second;
//    auto ee = boost::edge(v1, v2, g);
//    assert(ee.second);
//    auto label1 = l(v1);
//    auto label2 = l(v2);
//    assert(new_vertex == n_vertices());
//    boost::add_vertex(g);
//    auto a                      = new_vertex++;
//    g[a].alpha_expansion_vertex = true;
//    add_edge(v1, a, lambda * e(label1, alpha_label));
//    add_edge(v2, a, lambda * e(alpha_label, label2));
//    add_edge(a, dat.sink_vertex, lambda * e(label1, label2));
//    weights2[boost::edge(v1, a, g).first]              = weights[ee.first];
//    weights2[boost::edge(v2, a, g).first]              = weights[ee.first];
//    weights2[boost::edge(a, dat.sink_vertex, g).first] = weights[ee.first];
// }

// // Break edges
// for(const auto& x : to_remove) this->remove_edge(x.first, x.second);

//    return dat;
// }

// void This::undo_alpha_expansion_modifications() noexcept(false)
// {
// vector<vertex_descriptor> to_remove;
// const auto N         = n_vertices();
// auto& g              = pimpl_->g;
// auto& e_idx          = pimpl_->edge_idx;
// const auto& weights  = get(boost::edge_weight, g);
// const auto& weights2 = get(boost::edge_weight2, g);

// vector<uint64_t> buffer;

// to_remove.reserve(N);
// for(auto n = 0u; n < N; ++n)
//    if(g[n].alpha_expansion_vertex) to_remove.push_back(n);

// if(to_remove.size() == 0) return; // Nothing to undo

// if(to_remove.size() == 1) {
//    rm_vertex_helper(g, boost::vertex(to_remove[0], g), e_idx, buffer);
//    throw std::logic_error("alpha-expansion graph in invalid state");
// }

// const auto original_N = N - to_remove.size();

// // Remove "additional" nodes other than source and sink (requires
// relinking) while(to_remove.size() > 2) {
//    auto v1 = boost::vertex(to_remove.back(), g);
//    if(v1 != boost::vertex(n_vertices() - 1, g))
//       throw std::logic_error("alpha-expansion graph in invalid state");
//    boost::graph_traits<G>::out_edge_iterator ei, e_end;
//    tie(ei, e_end) = out_edges(v1, g);
//    auto arity     = std::distance(ei, e_end);
//    if(int(arity) != 3)
//       throw std::logic_error("alpha-expansion graph in invalid state");

//    auto u0 = boost::source(*(ei + 0), g);
//    auto u1 = boost::target(*(ei + 0), g);
//    auto s0 = boost::source(*(ei + 1), g);
//    auto s1 = boost::target(*(ei + 1), g);
//    auto n0 = boost::source(*(ei + 2), g);
//    auto n1 = boost::target(*(ei + 2), g);

//    auto restore_weight = weights2[*ei];

//    // two should be v1. The other two are the edge we need to recreate
//    if(n0 >= original_N || n1 >= original_N) {
//       // using u0, u1, s0, s1
//    } else if(s0 >= original_N || s1 >= original_N) {
//       swap(s0, n0);
//       swap(s1, n1);
//    } else {
//       swap(u0, n0);
//       swap(u1, n1);
//    }

//    auto uu = (u0 == v1) ? u1 : u0;
//    auto ss = (s0 == v1) ? s1 : s0;

//    if(g[uu].alpha_expansion_vertex | g[ss].alpha_expansion_vertex)
//       throw std::logic_error("alpha-expansion graph in invalid state");

//    // Build an edge between (uu and ss)
//    this->remove_edge(u0, u1);
//    this->remove_edge(s0, s1);
//    this->remove_edge(boost::source(*(ei + 2), g),
//                      boost::target(*(ei + 2), g));
//    this->add_edge(uu, ss);
//    weights2[boost::edge(uu, ss, g).first] = restore_weight;

//    rm_vertex_helper(g, boost::vertex(to_remove.back(), g), e_idx, buffer);
//    to_remove.pop_back();
// }

// // Remove source and sink -- no additional edges to relink
// while(to_remove.size() > 0) {
//    rm_vertex_helper(g, boost::vertex(to_remove.back(), g), e_idx, buffer);
//    to_remove.pop_back();
// }

// { // Zero out all weight2s
//    boost::graph_traits<G>::edge_iterator ei, ei_end;
//    for(tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
//       auto w2       = weights2[*ei];
//       weights[*ei]  = w2;
//       weights2[*ei] = 0.0;
//    }
// }
//}

// --------------------------------------------------------------------- min-cut

// Extract the min-cut from graph-flow

// static This::CutResult
// directed_cut(const G& g, unsigned source_id, unsigned sink_id)
// {
//    This::CutResult cut;
//    const unsigned N = boost::num_vertices(g);

//    typedef boost::
//        adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>
//            FTraits;
//    typedef boost::adjacency_list<
//        boost::vecS,
//        boost::vecS,
//        boost::directedS,
//        boost::property<
//            boost::vertex_index_t,
//            unsigned,
//            boost::property<
//                boost::vertex_color_t,
//                boost::default_color_type,
//                boost::property<boost::vertex_distance_t,
//                                double,
//                                boost::property<boost::vertex_predecessor_t,
//                                                FTraits::edge_descriptor>>>>,

//        boost::property<
//            boost::edge_capacity_t,
//            double,
//            boost::property<boost::edge_residual_capacity_t,
//                            double,
//                            boost::property<boost::edge_reverse_t,
//                                            FTraits::edge_descriptor>>>>
//        F;

//    using namespace boost;

//    typedef typename graph_traits<F>::vertex_descriptor f_vertex_descriptor;
//    typedef typename graph_traits<F>::edge_descriptor f_edge_descriptor;

//    F f(N);

//    // std::unordered_set<uint64_t> keys;
//    auto add_edge = [&](unsigned v1, unsigned v2, double weight) {
//       // if(keys.count(key(v1, v2)) > 0) return;
//       // keys.insert(key(v1, v2));
//       f_edge_descriptor e1, e2;
//       bool success1, success2;
//       boost::tie(e1, success1) = boost::add_edge(v1, v2, f);
//       boost::tie(e2, success2) = boost::add_edge(v2, v1, f);

//       if(!success1 || !success2) {
//          fprintf(stderr, "kBAM!!!\n");
//          exit(1);
//       }

//       const auto& capacity          = get(boost::edge_capacity, f);
//       const auto& residual_capacity = get(edge_residual_capacity, f);
//       const auto& reverse_edge      = get(boost::edge_reverse, f);
//       capacity[e1] = capacity[e2] = weight;
//       residual_capacity[e1] = residual_capacity[e2] = 0.0;
//       reverse_edge[e1]                              = e2;
//       reverse_edge[e2]                              = e1;
//    };

//    { // reserve edge space
//       for(auto n = 0u; n < N; ++n) {
//          const auto sz = g.m_vertices[n].m_out_edges.size();
//          f.m_vertices[n].m_out_edges.reserve(2 * sz);
//          // f.m_vertices[n].m_in_edges.reserve(sz);
//       }
//    }

//    {
//       const auto& weights = get(boost::edge_weight, g);
//       boost::graph_traits<G>::edge_iterator ei, e_end;
//       for(boost::tie(ei, e_end) = boost::edges(g); ei != e_end; ++ei)
//          add_edge(boost::source(*ei, g), boost::target(*ei, g),
//          weights[*ei]);
//    }

//    std::vector<default_color_type> color(num_vertices(f));
//    std::vector<long> distance(num_vertices(f));

//    // Find the flow
//    cut.flow = boykov_kolmogorov_max_flow(
//        f, boost::vertex(source_id, f), boost::vertex(sink_id, f));

//    const auto& colours = get(boost::vertex_color, f);
//    cut.source_set.resize(N);
//    std::fill(cut.source_set.begin(), cut.source_set.end(), false);
//    const auto black = boost::color_traits<F>::black();
//    for(auto n = 0u; n < N; ++n) cut.source_set[n] = (colours[n] == black);

//    return cut;
// }

This::CutResult This::min_cut(unsigned source_id, unsigned sink_id)
{
   auto& g = pimpl_->g;

   CutResult cut;
   cut.flow = boykov_kolmogorov_max_flow(
       g, boost::vertex(source_id, g), boost::vertex(sink_id, g));

   const unsigned N = n_vertices();
   cut.source_set.resize(N);
   std::fill(cut.source_set.begin(), cut.source_set.end(), false);
   const auto& colours = get(boost::vertex_color, g);
   const auto black    = boost::color_traits<G>::black();
   for(auto n = 0u; n < N; ++n) cut.source_set[n] = (colours[n] == black);

   return cut;
}

// ------------------------------------------------------------------- graph-viz

void This::write_graphviz(std::ostream& out) const
{
   using namespace boost;
   const auto& g = pimpl_->g;
   ::boost::write_graphviz(out, g, default_writer(), default_writer());
}

namespace detail
{
   template<typename Function> class functional_label_writer
   {
    private:
      Function f;

    public:
      functional_label_writer(Function func)
          : f(func)
      {}

      template<class VertexOrEdge>
      void operator()(std::ostream& out, const VertexOrEdge& v_or_e) const
      {
         out << f(v_or_e);
      }
   };
} // namespace detail

void This::write_graphviz(std::ostream& out,
                          vertex_label_function vert_labels,
                          edge_label_function edge_labels,
                          const CutResult* ret) const
{
   const auto& g = pimpl_->g;

   auto write_vertices = [&](std::ostream& out) {
      const auto N = n_vertices();
      for(auto n = 0u; n < N; ++n) {
         const bool in_source_set
             = (ret == nullptr ? false : ret->source_set[n]);
         out << "   " << n << " [label=\"" << vert_labels(n) << "\""
             << (in_source_set ? ", color=red" : "") << "];\n";
      }
   };

   auto write_edges = [&](std::ostream& out) {
      boost::graph_traits<G>::edge_iterator ei, ei_end;
      const auto& vindex  = get(boost::vertex_index, g);
      const auto& eweight = get(boost::edge_weight, g);

      std::array<char, 80> buffer;

      for(std::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
         int ind0 = int(boost::source(*ei, g));
         int ind1 = int(boost::target(*ei, g));

         if(ind0 < ind1) continue;

         const bool in0
             = (ret == nullptr ? false : ret->source_set[size_t(ind0)]);
         const bool in1
             = (ret == nullptr ? false : ret->source_set[size_t(ind1)]);

         const auto wgt = eweight(*ei);
         auto label     = edge_labels(unsigned(ind0), unsigned(ind1));
         if(label.empty()) {
            snprintf(&buffer[0], buffer.size(), "%g", double(wgt));
            label = &buffer[0];
         }

         out << "   " << ind0 << " -- " << ind1 << " [label=\"" << label
             << "\", weight=\"" << eweight(*ei) << "\"";
         if(in0 && in1) out << ", color=red";
         if(in0 != in1) out << ", color=gray";

         out << "];\n";
      }
   };

   out << "graph G {\n";
   write_vertices(out);
   out << "\n";
   write_edges(out);
   out << "}" << std::flush;
}

// Write out vertices

void This::save_graphviz(std::string filename,
                         vertex_label_function vlabels,
                         edge_label_function elabels,
                         const CutResult* ret) const
{
   std::ofstream ofs;
   ofs.open(filename);
   this->write_graphviz(ofs, vlabels, elabels, ret);
   ofs.close();
   std::stringstream ss("");
   ss << "dot -Tpng -O \"" << filename << "\"";
   auto cmd     = ss.str();
   auto success = system(cmd.c_str()) == EXIT_SUCCESS;
}

} // namespace perceive
