
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/utils/dag-node.hpp"

namespace perceive
{
class DAGNode
{
 private:
   string name_;
   vector<DAGNode*> parents_;  // memory is not owned
   vector<DAGNode*> children_; // memory is not owned

 public:
   DAGNode(string name = ""s)
       : name_(name)
   {}

   string label() const noexcept { return name_; }

   vector<DAGNode*>& parents() noexcept { return parents_; }
   vector<DAGNode*>& children() noexcept { return children_; }
   const vector<DAGNode*>& parents() const noexcept { return parents_; }
   const vector<DAGNode*>& children() const noexcept { return children_; }

   void subscribe(DAGNode* p) noexcept { dag::add_parent(this, p); }
   void unsubscribe(DAGNode* p) noexcept { dag::remove_parent(this, p); }

   bool has_parent(const DAGNode* p) const noexcept
   {
      return dag::has_parent(this, p);
   }

   bool has_child(const DAGNode* p) const noexcept
   {
      return dag::has_child(this, p);
   }

   bool has_ancestor(const DAGNode* p) const noexcept
   {
      return dag::has_ancestor(this, p);
   }

   bool has_descendent(const DAGNode* p) const noexcept
   {
      return dag::has_descendent(this, p);
   }

   auto find_all_connected_nodes() const noexcept
   {
      return dag::all_connected_to(this);
   }
};

CATCH_TEST_CASE("DagNode", "[dag_node]")
{
   //
   // ----------------------------------------------------------- TweakerParams
   //
   CATCH_SECTION("dag-node")
   {
      // CATCH_REQUIRE((p == q));
      DAGNode a{"a"}, b{"b"}, c{"c"}, d{"d"}, e{"e"};

      array<DAGNode*, 5> nodes;
      nodes[0] = &a;
      nodes[1] = &b;
      nodes[2] = &c;
      nodes[3] = &d;
      nodes[4] = &e;

      c.subscribe(&a);
      c.subscribe(&b);
      d.subscribe(&c);
      e.subscribe(&c);

      CATCH_REQUIRE(c.has_parent(&a));
      CATCH_REQUIRE(c.has_parent(&b));
      CATCH_REQUIRE(!c.has_parent(&c));
      CATCH_REQUIRE(!c.has_parent(&d));
      CATCH_REQUIRE(!c.has_parent(&e));

      CATCH_REQUIRE(!c.has_child(&a));
      CATCH_REQUIRE(!c.has_child(&b));
      CATCH_REQUIRE(!c.has_child(&c));
      CATCH_REQUIRE(c.has_child(&d));
      CATCH_REQUIRE(c.has_child(&e));

      CATCH_REQUIRE(!a.has_descendent(&a));
      CATCH_REQUIRE(!a.has_descendent(&b));
      CATCH_REQUIRE(a.has_descendent(&c));
      CATCH_REQUIRE(a.has_descendent(&d));
      CATCH_REQUIRE(a.has_descendent(&e));

      CATCH_REQUIRE(e.has_ancestor(&a));
      CATCH_REQUIRE(e.has_ancestor(&b));
      CATCH_REQUIRE(e.has_ancestor(&c));
      CATCH_REQUIRE(!e.has_ancestor(&d));
      CATCH_REQUIRE(!e.has_ancestor(&e));

      for(auto n : nodes)
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
      for(auto n : nodes) CATCH_REQUIRE(!dag::has_cycle(n));

      // -- cycle check
      b.subscribe(&e);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(dag::has_cycle(n));
      }
      b.unsubscribe(&e);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(!dag::has_cycle(n));
      }

      // -- cycle check
      e.subscribe(&b);
      e.subscribe(&d);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(!dag::has_cycle(n));
      }
      e.unsubscribe(&b);
      e.unsubscribe(&d);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(!dag::has_cycle(n));
      }

      // -- cycle check
      c.subscribe(&d);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(dag::has_cycle(n));
      }
      c.unsubscribe(&d);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(!dag::has_cycle(n));
      }

      // -- cycle check
      c.subscribe(&c);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(dag::has_cycle(n));
      }
      c.unsubscribe(&c);
      for(auto n : nodes) {
         CATCH_REQUIRE(n->find_all_connected_nodes().size() == 5);
         CATCH_REQUIRE(!dag::has_cycle(n));
      }

      // cout << dag::to_dot_graph(&a);
   }
}

} // namespace perceive
