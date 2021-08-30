
#pragma once

#include "node.hpp"

namespace perceive::tracks
{
class NodeSequence
{
 private:
   int id_ = -1; // id of track
   vector<Node> nodes_;

 public:
   NodeSequence() = default;
   NodeSequence(int id, vector<Node>&&) noexcept;
   NodeSequence(const NodeSequence&)     = default;
   NodeSequence(NodeSequence&&) noexcept = default;
   ~NodeSequence()                       = default;
   NodeSequence& operator=(const NodeSequence&) = default;
   NodeSequence& operator=(NodeSequence&&) noexcept = default;

   int id() const noexcept { return id_; }
   const vector<Node>& nodes() const noexcept { return nodes_; }
   size_t size() const noexcept { return nodes_.size(); }
   int n_frames() const noexcept;

   bool is_valid() const noexcept;
   size_t memory_usage() const noexcept;
   void check_invariants() const noexcept;

   void clip_nodes_to_t(const int max_t_inclusive) noexcept;
   void append(const NodeSequence& seq) noexcept;
   void prepend_missing(const NodeSequence& seq) noexcept;
   string to_string() const noexcept;
};

Track node_sequence_to_track(const NodeSequence& seq);
NodeSequence track_to_node_sequence(const Track& track);

template<typename InputIt>
vector<Track> node_sequence_to_tracks(InputIt begin, InputIt end)
{
   vector<Track> out;
   out.reserve(size_t(std::distance(begin, end)));
   while(begin != end) out.emplace_back(node_sequence_to_track(*begin++));
   return out;
}

template<typename InputIt>
vector<NodeSequence> tracks_to_node_sequence(InputIt begin, InputIt end)
{
   vector<NodeSequence> out;
   out.reserve(size_t(std::distance(begin, end)));
   while(begin != end) {
      const Track& tt = *begin++;
      out.emplace_back(track_to_node_sequence(tt));
   }
   return out;
}

} // namespace perceive::tracks
