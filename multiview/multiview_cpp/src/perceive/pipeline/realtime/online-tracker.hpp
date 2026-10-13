
#pragma once

#include "frame-record.hpp"
#include "track-messages.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/pipeline/tracker/tracker-results.hpp"

namespace perceive::pipeline::realtime
{
class RealtimeStats;

// --------------------------------------------------------------- OnlineTracker
// The batch tracker run on frames as they arrive, on a thread of its own.
// Frames come in frame order (`push`); every `max_frames_per_tracklet`
// frames close a window, which runs `Tracklet::execute` and the window
// function (by default `calc_tracks_comp_data(...)->execute()`) exactly as
// the `tracks` task does, with the previous window's `Tracks` to revise.
//
// Published per window: `tracks` revision 0 ("tracked") for the frames of the
// window; revision 1 ("revised") for the frames of the window before, which
// the window has just rewritten, with the ids that changed (`id_remap`);
// `smoothed` points for the tracks that got revised nodes; and `track_end`
// for the tracks that ended. At the end of the stream the last window is
// published as revised, and `finish` concatenates every window as the batch
// does (`make_tracker_result`, `finalise_tracks`), so a run without drops
// gives the batch output.
//
// Dropped frames are filled with `LocalizationData::make_empty` and the LAB
// images and point clouds of the last good frame. Windows before the first
// good frame are not tracked. A window that fails validation is skipped,
// a `window_error` is published, and the tracker stops tracking
// (`has_error`): nothing on its thread calls `exit`.
//
class OnlineTracker
{
 public:
   struct Params
   {
      Tracklet::Params tracklet_params;
      Tracks::Params tracks_params;
      PostProcessParams post_process;
      fowlkes::Params fowlkes_params;
      LocalizationData::Params loc_params;
      int start_frame     = 0;
      int end_frame       = 0; // exclusive
      real frame_duration = 1.0 / 15.0;
      string outdir       = "/tmp"s;
   };

   // Called on the tracker thread
   using publish_fn  = std::function<void(TrackMessage&&)>;
   using window_fn   = std::function<Tracks(const Tracklet* tracklet,
                                          const Tracklet* prev_tracklet,
                                          Tracks* prev)>;
   using observer_fn = std::function<void(const Tracks& newest)>;

 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   // An empty `window` runs the batch window code on `scene`. `stats` may be
   // null; it must outlive the tracker.
   OnlineTracker(shared_ptr<const SceneDescription> scene,
                 const Params& p,
                 publish_fn publish,
                 window_fn window     = {},
                 observer_fn observer = {},
                 RealtimeStats* stats = nullptr);
   OnlineTracker(const OnlineTracker&) = delete;
   OnlineTracker(OnlineTracker&&)      = delete;
   ~OnlineTracker(); // stops and joins
   OnlineTracker& operator=(const OnlineTracker&) = delete;
   OnlineTracker& operator=(OnlineTracker&&)      = delete;

   const Params& params() const noexcept;
   int frames_per_window() const noexcept;
   int delta_t() const noexcept;    // smoothing lag, in frames
   bool has_error() const noexcept; // a window failed
   size_t queue_size() const noexcept;
   int n_windows() const noexcept; // tracked so far

   // Valid after `finish`
   int n_frames() const noexcept; // frames pushed
   const TrackerResult& raw_result() const noexcept;
   const FowlkesResult& presmooth_result() const noexcept;
   const FowlkesResult& final_result() const noexcept;

   bool start() noexcept;

   // Blocks while the queue (one window) is full. False after `finish`.
   bool push(FrameRecord&& record) noexcept;

   // Closes the queue, lets the thread publish the last window, joins it,
   // and makes the results (on the calling thread). Call once.
   void finish() noexcept;
};

} // namespace perceive::pipeline::realtime
