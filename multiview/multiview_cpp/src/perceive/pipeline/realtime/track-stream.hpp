
#pragma once

#include "perceive/cost-functions/tracks/node-sequence.hpp"
#include "perceive/optimization/kz-filter.hpp"

#include <map>

namespace perceive
{
class Tracks;
}

namespace perceive::pipeline::realtime
{
// -------------------------------------------------------------- SmoothedUpdate
// The smoothed points of one track from the first point that was not final
// before, to the track's last revised node. Points with t <= `final_upto_t`
// are final: they equal the batch result for the track.
//
struct SmoothedUpdate
{
   int id           = -1;
   int final_upto_t = -1;
   bool is_ended    = false;
   Track track; // `path` holds the points, in histogram cells
};

// ----------------------------------------------------------------- TrackStream
// The revised nodes of every track so far, by id, and their smoothing with
// the batch's own `smooth_track`. A window is revised once the next window
// has run; its nodes are then final, and a track's smoothed point at t is
// final once the nodes up to t + delta_t are, or once the track has ended.
// Not thread-safe: the tracker thread owns it.
//
class TrackStream
{
 private:
   struct Stream
   {
      tracks::NodeSequence seq;
      int published_upto_t = -1; // points <= this were published as final
      bool is_ended        = false;
   };

   KzFilter filter_;
   bool is_smoothing_ = false;
   std::map<int, Stream> streams_;

   SmoothedUpdate make_update_(Stream& stream, int final_upto_t);

 public:
   // No smoothing if `kz_m` or `kz_k` is not positive, as `smooth_tracks`
   TrackStream(int kz_m, int kz_k);
   TrackStream(const TrackStream&)            = delete;
   TrackStream(TrackStream&&)                 = default;
   ~TrackStream()                             = default;
   TrackStream& operator=(const TrackStream&) = delete;
   TrackStream& operator=(TrackStream&&)      = default;

   // Frames of smoothing lag: 0 without smoothing
   int delta_t() const noexcept;
   size_t n_tracks() const noexcept { return streams_.size(); }

   // The revised nodes of track `id` so far; nullptr if there are none
   const tracks::NodeSequence* sequence(int id) const noexcept;
   bool is_ended(int id) const noexcept;

   // Appends the revised window `revised`, whose last frame is
   // `last_revised_t`. A track that is not in `newest` (the window after
   // `revised`; nullptr at the end of the stream) has ended. Returns an
   // update for every track that got nodes or ended. A sequence that does
   // not continue its track's nodes is skipped and logged.
   vector<SmoothedUpdate> update(const Tracks& revised,
                                 const Tracks* newest,
                                 int last_revised_t) noexcept;
};

} // namespace perceive::pipeline::realtime
