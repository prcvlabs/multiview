
#include "track-stream.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"

#include <unordered_set>

namespace perceive::pipeline::realtime
{
using tracks::Node;
using tracks::NodeSequence;

// ---------------------------------------------------------------- Construction
//
TrackStream::TrackStream(int kz_m, int kz_k)
    : is_smoothing_(kz_m > 0 and kz_k > 0)
{
   if(is_smoothing_) filter_.init(kz_m, kz_k);
}

// --------------------------------------------------------------------- delta-t
//
int TrackStream::delta_t() const noexcept
{
   return is_smoothing_ ? filter_.delta_t() : 0;
}

// -------------------------------------------------------------------- sequence
//
const NodeSequence* TrackStream::sequence(int id) const noexcept
{
   auto ii = streams_.find(id);
   return (ii == cend(streams_)) ? nullptr : &ii->second.seq;
}

// -------------------------------------------------------------------- is-ended
//
bool TrackStream::is_ended(int id) const noexcept
{
   auto ii = streams_.find(id);
   return (ii != cend(streams_)) and ii->second.is_ended;
}

// ----------------------------------------------------------------- make-update
// Smooths the tail of the track that holds every point not yet published as
// final, plus `delta_t` points before it: a smoothed point reads only the
// samples within `delta_t` of it, so the tail gives the same values as the
// whole track.
//
SmoothedUpdate TrackStream::make_update_(Stream& stream, int final_upto_t)
{
   const auto& nodes = stream.seq.nodes();

   SmoothedUpdate update;
   update.id           = stream.seq.id();
   update.final_upto_t = final_upto_t;
   update.is_ended     = stream.is_ended;
   update.track.id     = stream.seq.id();
   if(nodes.empty()) return update;

   const int t0       = nodes.front().t();
   const int first_t  = std::max(t0, stream.published_upto_t + 1);
   const int i0       = first_t - t0;
   const int tail_pos = std::max(0, i0 - delta_t());
   if(i0 >= int(nodes.size())) return update;

   const auto tail_begin = std::next(cbegin(nodes), tail_pos);
   Track tt              = node_sequence_to_track(
       NodeSequence(stream.seq.id(), vector<Node>(tail_begin, cend(nodes))));
   if(is_smoothing_) smooth_track(filter_, tt);

   update.track.path.assign(std::next(cbegin(tt.path), i0 - tail_pos),
                            cend(tt.path));
   stream.published_upto_t = std::max(stream.published_upto_t, final_upto_t);
   return update;
}

// ---------------------------------------------------------------------- update
//
vector<SmoothedUpdate> TrackStream::update(const Tracks& revised,
                                           const Tracks* newest,
                                           int last_revised_t) noexcept
{
   vector<SmoothedUpdate> out;
   try {
      std::unordered_set<int> newest_ids;
      if(newest != nullptr)
         for(const auto& seq : newest->seqs) newest_ids.insert(seq.id());

      // Append the revised nodes
      vector<int> touched;
      for(const auto& seq : revised.seqs) {
         if(seq.size() == 0) continue;
         auto& stream      = streams_[seq.id()];
         const auto& nodes = stream.seq.nodes();
         if(nodes.size() > 0
            and nodes.back().t() + 1 != seq.nodes().front().t()) {
            LOG_ERR(format("track {} does not continue at frame {} (its last "
                           "node is at {}); skipping its nodes",
                           seq.id(),
                           seq.nodes().front().t(),
                           nodes.back().t()));
            continue;
         }
         if(stream.is_ended) {
            WARN(format("track {} continues after it ended", seq.id()));
            stream.is_ended = false;
         }

         if(nodes.size() == 0)
            stream.seq = seq;
         else
            stream.seq.append(seq);
         touched.push_back(seq.id());
      }

      // Every live track that is not in the newest window has ended
      for(auto& [id, stream] : streams_) {
         if(stream.is_ended or newest_ids.count(id) > 0) continue;
         stream.is_ended = true;
         if(std::find(cbegin(touched), cend(touched), id) == cend(touched))
            touched.push_back(id);
      }

      std::sort(begin(touched), end(touched));
      for(const int id : touched) {
         auto& stream = streams_.at(id);
         if(stream.seq.size() == 0) continue;
         const int last_t = stream.seq.nodes().back().t();
         const int final_upto_t
             = stream.is_ended ? last_t
                               : std::min(last_t, last_revised_t - delta_t());
         out.push_back(make_update_(stream, final_upto_t));
      }
   } catch(std::exception& e) {
      LOG_ERR(format("exception updating the track stream: {}", e.what()));
   }
   return out;
}

} // namespace perceive::pipeline::realtime
