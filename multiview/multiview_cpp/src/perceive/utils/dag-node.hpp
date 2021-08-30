
#pragma once

#include <algorithm>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace perceive
{
// -----------------------------------------------------------------------------
// Directed Graph
//
// Concept:
//
//    class Node {
//       string graph_label() const noexcept;
//       container<const Node *>& children() noexcept;
//       container<const Node *>& parents() noexcept;
//       const container<const Node *>& children() const noexcept;
//       const container<const Node *>& parents() const noexcept;
//    };
//
// Operations:
//
//    void add_parent(o, p) noexcept
//    void add_child(o, p) noexcept
//
//    void remove_parent(o, p) noexcept
//    void remove_child(o, p) noexcept
//
//    void has_parent(o, p) noexcept
//    void has_child(o, p) noexcept
//
//    void has_ancestor(o, p) noexcept       // Precondition: no cycles
//    void has_descendent(o, p) noexcept     // Precondition: no cycles
//
//    void visit_ancestors(o, f) noexcept    // Precondition: no cycles
//    void visit_descendents(o, f) noexcept  // Precondition: no cycles
//
//    vector<T> all_connected_to(o) noexcept
//    bool has_cycle(o, f) noexcept
//
//    string to_str(o) noexcept
//    string to_dot_graph(o) noexcept
//
namespace dag
{
   template<typename Node> inline void add_parent(Node* o, Node* p) noexcept;
   template<typename Node> inline void add_child(Node* o, Node* p) noexcept;

   template<typename Node> inline void remove_parent(Node* o, Node* p) noexcept;
   template<typename Node> inline void remove_child(Node* o, Node* p) noexcept;

   template<typename Node>
   inline bool has_parent(const Node* o, const Node* p) noexcept;

   template<typename Node>
   inline bool has_child(const Node* o, const Node* p) noexcept;

   // Precondition: no cycles
   template<typename Node>
   inline bool has_ancestor(const Node* o, const Node* p) noexcept;

   // Precondition: no cycles
   template<typename Node>
   inline bool has_descendent(const Node* o, const Node* p) noexcept;

   // Precondition: no cycles
   // "f" returns FALSE to short-circuit visit operation
   // Returns TRUE iff all visit operations return TRUE
   template<typename Node>
   inline bool visit_ancestors(Node* o,
                               std::function<bool(Node* x)> f) noexcept;

   // Precondition: no cycles
   // "f" returns FALSE to short-circuit visit operation
   // Returns TRUE iff all visit operations return TRUE
   template<typename Node>
   inline bool visit_descendents(Node* o,
                                 std::function<bool(Node* x)> f) noexcept;

   // Cycle "safe"
   template<typename Node>
   inline std::vector<const Node*> all_connected_to(const Node* o) noexcept;

   // Cycle "safe"
   template<typename Node> inline bool has_cycle(const Node* node) noexcept;

   // Nice looking string
   template<typename Node> inline std::string to_str(const Node* o) noexcept;

   /**
    * Save this string to file, and then:
    * > sudo apt-get install graphviz
    * > dot -Tpng my-file > foo.png
    */
   template<typename Node>
   inline std::string to_dot_graph(const Node* node) noexcept;
} // namespace dag

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
// -------------------------------------------------------------- Implementation
// Directed Acyclic Graph (DAG)
namespace dag
{
   namespace detail
   {
      template<typename Container, typename Value>
      inline void erase_it(Container& container, Value val)
      {
         while(true) {
            auto ii = find(begin(container), end(container), val);
            if(ii != end(container))
               container.erase(ii);
            else
               break;
         }
      }
   } // namespace detail

   // -------------------------------------------------- add/remove parent/child
   template<typename Node> inline void add_parent(Node* o, Node* p) noexcept
   {
      o->parents().insert(end(o->parents()), p);
      p->children().insert(end(p->children()), o);
   }

   template<typename Node> inline void remove_parent(Node* o, Node* p) noexcept
   {
      detail::erase_it(o->parents(), p);
      detail::erase_it(p->children(), o);
   }

   template<typename Node> inline void add_child(Node* o, Node* p) noexcept
   {
      o->children().insert(end(o->children()), p);
      p->parents().insert(end(p->parents()), o);
   }

   template<typename Node> inline void remove_child(Node* o, Node* p) noexcept
   {
      detail::erase_it(o->children(), p);
      detail::erase_it(p->parents(), o);
   }

   // --------------------------------------------------------- has parent/child
   template<typename Node>
   inline bool has_parent(const Node* o, const Node* p) noexcept
   {
      return find(cbegin(o->parents()), cend(o->parents()), p)
             != cend(o->parents());
   }

   template<typename Node>
   inline bool has_child(const Node* o, const Node* p) noexcept
   {
      return find(cbegin(o->children()), cend(o->children()), p)
             != cend(o->children());
   }

   // ------------------------------------------------ visit ancestor/descendent
   template<typename Node>
   inline bool visit_ancestors(Node* o, std::function<bool(Node* x)> f) noexcept
   {
      return all_of(cbegin(o->parents()), cend(o->parents()), [&](Node* x) {
         return f(x) and visit_ancestors(x, f);
      });
   }

   template<typename Node>
   inline bool visit_descendents(Node* o,
                                 std::function<bool(Node* x)> f) noexcept
   {
      return all_of(cbegin(o->children()), cend(o->children()), [&](Node* x) {
         return f(x) and visit_descendents(x, f);
      });
   }

   // -------------------------------------------------- has ancestor/descendent
   template<typename Node>
   inline bool has_ancestor(const Node* o, const Node* p) noexcept
   {
      std::function<bool(const Node*)> f
          = [&](const Node* x) -> bool { return x != p; };
      return !visit_ancestors(o, f);
   }

   template<typename Node>
   inline bool has_descendent(const Node* o, const Node* p) noexcept
   {
      std::function<bool(const Node*)> f
          = [&](const Node* x) -> bool { return x != p; };
      return !visit_descendents(o, f);
   }

   // ------------------------------------------------- find-all-connected-nodes
   template<typename Node>
   inline std::vector<const Node*> all_connected_to(const Node* o) noexcept
   {
      std::unordered_set<const Node*> visited;

      std::function<void(const Node*)> find_pc = [&](const Node* p) {
         if(visited.count(p) != 0) return;
         visited.insert(p);
         for(auto x : p->parents()) find_pc(x);
         for(auto x : p->children()) find_pc(x);
      };

      find_pc(o);
      return std::vector<const Node*>(begin(visited), end(visited));
   }

   // ---------------------------------------------------------------- has-cycle
   template<typename Node> inline bool has_cycle(const Node* node) noexcept
   {
      auto all_nodes = all_connected_to(node);
      auto visited   = std::vector<bool>(all_nodes.size());
      auto in_stack  = std::vector<bool>(all_nodes.size());
      std::fill(begin(visited), end(visited), false);
      std::fill(begin(in_stack), end(in_stack), false);

      auto lookup = std::unordered_map<const Node*, unsigned>();
      lookup.reserve(all_nodes.size());
      for(auto i = 0u; i < all_nodes.size(); ++i) lookup[all_nodes[i]] = i;

      std::function<bool(unsigned i)> is_cyclic = [&](unsigned i) -> bool {
         if(!visited[i]) {
            visited[i]  = true;
            in_stack[i] = true; // add to 'stack'

            for(auto child : all_nodes[i]->children()) {
               auto ii = lookup.find(child);
               Expects(ii != end(lookup));
               auto ind = ii->second;
               if(in_stack[ind] or is_cyclic(ind)) return true;
            }

            in_stack[i] = false; // remove from 'stack'
         }

         return false; // no cycle found (so far)
      };

      for(auto i = 0u; i < all_nodes.size(); ++i)
         if(is_cyclic(i)) return true;
      return false;
   }

   // ------------------------------------------------------------------- to-str
   template<typename Node> inline string to_str(const Node* o) noexcept
   {
      auto label = [](const Node* x) { return x->graph_label(); };
      return format(
          "[{}]  -->  {}  -->  [{}]",
          implode(cbegin(o->parents()), cend(o->parents()), ", ", label),
          label(o),
          implode(cbegin(o->children()), cend(o->children()), ", ", label));
   }

   // ------------------------------------------------------------- to-dot-graph
   template<typename Node> inline string to_dot_graph(const Node* node) noexcept
   {
      auto all_nodes = all_connected_to(node);

      // setup the labels
      vector<string> labels;
      labels.resize(all_nodes.size());
      auto task_label = [](const Node* x) {
         return x->graph_label().empty()
                    ? format("{:p}", reinterpret_cast<const void*>(x))
                    : x->graph_label();
      };

      std::transform(
          cbegin(all_nodes), cend(all_nodes), begin(labels), task_label);

      // setup lookup: label  -->  index
      std::unordered_map<string, int> lookup;
      for(size_t i = 0; i < all_nodes.size(); ++i) lookup[labels[i]] = int(i);

      std::stringstream ss("");

      ss << "digraph DAG {" << endl;

      auto encode_label = [&](const string& s) {
         std::stringstream ss("");
         for(auto c : s) {
            switch(c) {
            case '"': ss << "\\\""; break;
            case '\\': ss << "\\\\"; break;
            case ' ': ss << "\\ "; break;
            case '[': ss << "\\["; break;
            case ']': ss << "\\]"; break;
            case '(': ss << "\\("; break;
            case ')': ss << "\\)"; break;
            case '{': ss << "\\{"; break;
            case '}': ss << "\\}"; break;
            case '-': ss << c; break;
            case '_': ss << c; break;
            default:
               if(!std::isalnum(c))
                  FATAL(format(
                      "invalid character '{:c}' in graph-label '{}'", c, s));
               ss << c;
            }
         }

         return ss.str();
      };

      // Output all nodes
      for(auto i = 0u; i < all_nodes.size(); ++i) {
         ss << format(
             "   node_{} [label = \"{}\"];\n", i, encode_label(labels[i]));
      }

      // Output all edges
      for(auto i = 0u; i < all_nodes.size(); ++i) {
         auto p        = all_nodes[i];
         string src_id = format("node_{}", i);
         for(auto x : p->children()) {
            string dst_id = format("node_{}", lookup[task_label(x)]);
            ss << format("   {} -> {};\n", src_id, dst_id);
         }
      }

      ss << "}";

      return ss.str();
   }

} // namespace dag

} // namespace perceive
