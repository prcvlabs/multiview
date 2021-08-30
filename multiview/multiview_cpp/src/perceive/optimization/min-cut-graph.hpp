
#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace perceive
{
using std::vector;

class MinCutGraph
{
 private:
   class Pimpl;
   std::unique_ptr<Pimpl> pimpl_;

 public:
   MinCutGraph(unsigned n_verts = 0);
   MinCutGraph(MinCutGraph&&) noexcept;
   MinCutGraph(const MinCutGraph&);
   ~MinCutGraph();
   MinCutGraph& operator=(MinCutGraph&&) noexcept;
   MinCutGraph& operator=(const MinCutGraph&);

   // -- Set the number of vertices
   void reserve_vertices(unsigned n_verts);
   void reserve_edges(unsigned vert_ind, unsigned n_edges);
   void set_n_vertices(unsigned n_verts);
   unsigned n_vertices() const noexcept;
   unsigned n_edges() const noexcept;
   unsigned size() const noexcept;

   // -- Managing edges (symantics are: undirected edges)
   bool has_edge(unsigned v1, unsigned v2) const noexcept;
   bool add_edge(unsigned v1, unsigned v2, double weight = 0.0);
   void remove_edge(unsigned v1, unsigned v2);
   void remove_all_edges();

   // -- Edge weights
   // float weight(unsigned v1, unsigned v2) const noexcept;
   // bool set_weight(unsigned v1, unsigned v2, double weight = 0.0) noexcept;

   // -- Prepare for alpha expansion
   using label_of_vertex_func   = std::function<unsigned(unsigned)>;
   using vertex_label_cost_func = std::function<double(unsigned, unsigned)>;
   using label_label_cost_func  = std::function<double(unsigned, unsigned)>;
   // using vlabel_vlabel_cost_func =std::function<double(unsigned, unsigned,
   //                                                     unsigned, unsigned)>;
   // struct AlphaExpansionData
   // {
   //    unsigned alpha_label{0};
   //    unsigned source_vertex{0};
   //    unsigned sink_vertex{0};
   // };
   // // NOTE: if vertex-label and label-label are expensive operations, then
   // // memoize the results of these functions.
   // AlphaExpansionData modify_for_alpha_expansion(unsigned alpha_label,
   //                                               label_of_vertex_func,
   //                                               vertex_label_cost_func,
   //                                               label_label_cost_func,
   //                                               double smoothing_lambda);
   // AlphaExpansionData modify_for_alpha_expansion(unsigned alpha_label,
   //                                               label_of_vertex_func,
   //                                               vertex_label_cost_func,
   //                                               vlabel_vlabel_cost_func,
   //                                               double smoothing_lambda);
   // Throws an exception if the graph has since been modified
   // and will not restore edge weigths
   // void undo_alpha_expansion_modifications() noexcept(false);

   // -- Perform the graph cut
   struct CutResult
   {
      double flow{0.0};
      vector<bool> source_set; // bits set to TRUE for verts in source-set
   };
   CutResult min_cut(unsigned source_id, unsigned sink_id);

   // -- Print
   void write_graphviz(std::ostream& out) const;

   using vertex_label_function = std::function<std::string(unsigned)>;
   using edge_label_function   = std::function<std::string(unsigned, unsigned)>;
   void write_graphviz(std::ostream& out,
                       vertex_label_function vert_labels,
                       edge_label_function edge_labels,
                       const CutResult* ret = nullptr) const;

   void save_graphviz(std::string filename,
                      vertex_label_function vert_labels,
                      edge_label_function edge_labels,
                      const CutResult* ret = nullptr) const;
};

} // namespace perceive
