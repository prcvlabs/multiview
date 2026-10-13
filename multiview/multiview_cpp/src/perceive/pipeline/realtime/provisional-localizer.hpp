
#pragma once

#include "perceive/geometry/skeleton/skeleton-2d-info.hpp"

namespace perceive
{
struct LocalizationData;
class SceneDescription;
class Tracks;
} // namespace perceive

namespace perceive::pipeline::realtime
{
// --------------------------------------------------------- ProvisionalPosition
// Where a person probably is in one frame, before any tracking window closes
//
struct ProvisionalPosition
{
   Vector2 X        = {};       // metres
   Point2 hist_xy   = {-1, -1}; // histogram cell
   int n_detections = 0;        // 2D detections, across sensors
   float score      = 0.0f;     // `Skeleton2DInfoPosition::score`
   float prob_fp    = 1.0f;     // mean over the detections
   int pid          = -1;       // provisional id, -1 when unmatched
};

// ------------------------------------------------------------------- LiveTrack
// The last tracked position of a track, for labelling provisional positions
//
struct LiveTrack
{
   int id           = -1;
   int t            = -1; // frame of `X`
   Vector2 X        = {}; // metres
   Vector2 velocity = {}; // metres per frame
};

// Groups 2D detections by the peak (argmax) of their `prob_xy`: a detection
// joins the nearest group whose first detection's peak is within `diameter`
// metres, strongest peaks first. Returns indices into `infos`. Detections
// without a `prob_xy` are left out. Needs no scene.
vector<vector<int>>
cluster_detections(const vector<const Skeleton2DInfo*>& infos,
                   real hist_sz,
                   real diameter) noexcept;

// One position per group of `cluster_detections`, placed by the tracker's own
// estimator `position_p2d_infos`. Unlabelled (`pid == -1`).
vector<ProvisionalPosition> provisional_positions(const SceneDescription& scene,
                                                  const LocalizationData& loc,
                                                  real diameter) noexcept;

// The last node of every sequence of a tracking window
vector<LiveTrack> live_tracks(const Tracks& window) noexcept;

// --------------------------------------------------------- ProvisionalLabeller
// Labels provisional positions with the id of the nearest live track, one to
// one (`hungarian_algorithm`), within `match_radius` of where the track is
// expected at that frame. Tracks last seen more than `max_age` frames away
// are ignored. Thread-safe: the tracker updates, the publisher labels.
//
class ProvisionalLabeller
{
 private:
   mutable std::mutex padlock_;
   vector<LiveTrack> tracks_;
   real match_radius_ = 0.6;
   int max_age_       = 15;

 public:
   ProvisionalLabeller(real match_radius, int max_age);
   ProvisionalLabeller(const ProvisionalLabeller&)            = delete;
   ProvisionalLabeller(ProvisionalLabeller&&)                 = delete;
   ~ProvisionalLabeller()                                     = default;
   ProvisionalLabeller& operator=(const ProvisionalLabeller&) = delete;
   ProvisionalLabeller& operator=(ProvisionalLabeller&&)      = delete;

   real match_radius() const noexcept { return match_radius_; }
   int max_age() const noexcept { return max_age_; }
   vector<LiveTrack> tracks() const noexcept;
   void set_tracks(vector<LiveTrack> tracks) noexcept;

   void label(int frame_no,
              vector<ProvisionalPosition>& positions) const noexcept;
};

} // namespace perceive::pipeline::realtime
