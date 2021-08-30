
#include "node-sequence.hpp"

#define This NodeSequence

namespace perceive::tracks
{
This::NodeSequence(int id, vector<Node>&& nodes) noexcept
    : id_(id)
    , nodes_(std::move(nodes))
{}

int This::n_frames() const noexcept
{
   return (size() == 0) ? 0 : (nodes().back().t() - nodes().front().t() + 1);
}

// ------------------------------------------------------------ check-invariants
//
void This::check_invariants() const noexcept
{
   bool has_error = false;
   if(id() == -1) {
      has_error = (nodes().size() != 0);
   } else {
      if(id() < 0) has_error = true;
      if(size() > 1000000)
         FATAL(format("that's a very large track!, sz = {}", size()));
      for(size_t i = 1; i < size(); ++i)
         if(nodes()[i - 1].t() + 1 != nodes()[i].t()) has_error = true;
      for(const auto& node : nodes())
         if(node.p2ds().size() > 1000000)
            FATAL(format("corrupted, node.p2ds().size() = {}",
                         node.p2ds().size()));
   }

   if(has_error) {
      FATAL(format("error in node-sequence invariant:\n{}", to_string()));
   }
}

bool This::is_valid() const noexcept { return id_ >= 0; }

size_t This::memory_usage() const noexcept
{
   return sizeof(This) + vector_memory_usage(nodes_, [](const auto& o) {
             return o.memory_usage();
          });
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   return format("Track #{:-4d}\n   [{}]\n",
                 id(),
                 implode(cbegin(nodes()),
                         cend(nodes()),
                         ",\n    ",
                         [&](const auto& node) { return node.to_string(); }));
}

// ------------------------------------------------------------- clip-nodes-to-t
//
void This::clip_nodes_to_t(const int max_t_inclusive) noexcept
{
   if(size() == 0) return;
   const int t0 = nodes_.front().t();
   int idx      = -1;
   for(auto i = 0u; i < nodes_.size(); ++i) {
      Expects(nodes_[i].t() == t0 + int(i));
      if(idx == -1 && nodes_[i].t() > max_t_inclusive) idx = int(i);
   }
   if(idx >= 0) nodes_.erase(std::next(begin(nodes_), idx), end(nodes_));
}

// ---------------------------------------------------------------------- append
//
void This::append(const NodeSequence& seq) noexcept
{
   Expects(seq.id() == id());
   nodes_.insert(end(nodes_), cbegin(seq.nodes()), cend(seq.nodes()));
   check_invariants();
}

// ------------------------------------------------------------- prepend-interps
//
void This::prepend_missing(const NodeSequence& seq) noexcept
{
   Expects(seq.id() == id());
   if(seq.nodes().size() == 0) return; // nothing to do
   if(size() == 0) {
      nodes_.insert(end(nodes_), cbegin(seq.nodes()), cend(seq.nodes()));
      return; // There were all missing!
   }

   const int front_t = nodes().front().t();

   auto ss = cbegin(seq.nodes());
   auto ee = ss;
   while(ee != cend(seq.nodes()) && ee->t() < front_t) ++ee;

   nodes_.insert(begin(nodes()), ss, ee);
}

// ------------------------------------------------------ node-sequence-to-track
//
Track node_sequence_to_track(const NodeSequence& seq)
{
   Track tt;

   tt.id = seq.id();
   tt.path.resize(seq.nodes().size());
   std::transform(begin(seq.nodes()),
                  end(seq.nodes()),
                  begin(tt.path),
                  [](const Node& o) -> TrackPoint {
                     auto tp = TrackPoint{o.xy().x, o.xy().y, o.t(), o.gaze()};
                     tp.set_pose_scores(o.pose_scores());
                     return tp;
                  });
   tt.cost   = 0.0f; // TODO, integrate `seq` costs
   tt.height = 0.0f;
   tt.label  = Track::UNLABELLED;

   return tt;
}

// ------------------------------------------------------ track-to-node-sequence
//
NodeSequence track_to_node_sequence(const Track& track)
{
   vector<Node> nodes;
   nodes.resize(track.path.size());
   std::transform(cbegin(track.path),
                  cend(track.path),
                  begin(nodes),
                  [](const TrackPoint& tp) -> Node {
                     Node o;
                     o.set_xy(tp.xy());
                     o.set_t(tp.t);
                     o.set_gaze(tp.gaze_direction);
                     o.set_pose_scores(tp.pose_scores);
                     return o;
                  });

   return NodeSequence(track.id, std::move(nodes));
}

} // namespace perceive::tracks
