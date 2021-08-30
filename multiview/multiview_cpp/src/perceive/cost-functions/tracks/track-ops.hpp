
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/track.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"

// ------------------------------------------------------------ Track Operations
// [1] Merge paths at tracklets endpoints
// [2] Merge overlapping paths
// [3] Interpolate short gaps in paths
// [4] Smooth paths
// [5] Removes very short paths

namespace perceive::track_ops::detail
{
// Pretty-print a vector of tracks
string pp_tracks(const vector<Track>& tts) noexcept;

// Just temporal overlap
bool tracks_overlap_in_time(const Track& A, const Track& B) noexcept;

// Intersecting 'path' segment of 'A' and 'B'
vector<TrackPoint> track_intersection(const vector<TrackPoint>& A,
                                      const vector<TrackPoint>& B,
                                      const real overlap_threshold) noexcept;

// Clips the timepoints of a track to [start_frame, start_frame + n_frames)
void clamp_track(const int start_frame,
                 const int n_frames,
                 Track& track) noexcept;

// Split a single track around timepoint 't'. The first track has
// all timepoints STRICTLY less than t
std::pair<Track, Track> split_track(const Track& p, const int t) noexcept;

// Splits a vector of tracks into two vectors of tracks, at a given timestamp
std::pair<vector<Track>, vector<Track>> split_tracks(const vector<Track>& A,
                                                     const int t) noexcept;

// Split 'A' into 'N' parts covering 'env_size' time-stamps starting at 't0'
vector<vector<Track>> split_tracks(const vector<Track>& A,
                                   const int t0,
                                   const int env_size,
                                   const int N) noexcept;

// Merge a collection of tracks into a single collection of tracks
vector<Track> merge_track_sets(const vector<vector<Track>>& tracks) noexcept;

// Merge the ends of tracks in 'A' onto the beginnings of tracks in 'B'
// GREEDY algorithm
vector<Track>
merge_track_endpoints(const vector<Track>& A,
                      const vector<Track>& B,
                      const real merge_xy_dist_threshold) noexcept;

// A gap of 1 frame in track detection causes the creation of
// a brand new track. Interpolation closes these gaps.
// @param 'A' the collection of Tracks (which are destroyed)
// @param 'delta_xy_per_t' xy-distance that can be tolerated by delta-t
// @param 'max_delta_t' max t distance allowed for interpolation
vector<Track> interpolate_gaps_in_tracks(const vector<Track>& A,
                                         const real min_delta_xy,
                                         const real max_delta_xy_per_t,
                                         const int max_delta_t) noexcept;

// Two tracks can be in the following states:
// [1] A and B are disjoint (the paths never overlap)
// [2] A and B can cross for a period of time
// In the later case, we want to split A and B into multiple disjoint tracks
vector<Track>
decompose_overlapping_tracks(const vector<Track>& A,
                             const real overlap_threshold) noexcept;

// Sometimes tracks spatially overlap, at the beginning/ends and can be merged.
// Issue arises because of slight errors in calibration
// GREEDY algorithm
// NOTE: this algorithm will be deprecated (eventually) in favour of
// 'decompose-overlapping-tracks'
vector<Track>
merge_beginning_end_overlap_tracks(const vector<Track>& in,
                                   const real overlap_threshold) noexcept;

// (GREEDILY) removes any tracks that overlap
// The result is sorted by track start-time
vector<Track> remove_overlapping_tracks(const vector<Track>& in,
                                        const real overlap_threshold) noexcept;

// Smooth the path of each track
vector<Track> smooth_tracks(const vector<Track>& A) noexcept;

// Remove tracks whose length is below a threshold
// HOWEVER, do not remove starts that start at 'start-frame'
vector<Track> remove_short_tracks(const vector<Track>& A,
                                  const int min_size,
                                  const int start_frame) noexcept;

// -----------------------------------------------------------------------------
// --                                                    Additional Functions --
// -----------------------------------------------------------------------------
Tracks tracklets_to_tracks(const Tracks::Params& p,
                           const Tracklet& tracklet,
                           const int max_id_so_far) noexcept;

Tracks join_tracks(const Tracks& A, const Tracks& B) noexcept;

Tracks calc_tracks_ops_method(
    const Tracks::Params& p,
    const real frame_duration,
    const Tracklet* current_tracklet,
    Tracks* previous_track,
    std::function<const LocalizationData*(int t)> get_loc,
    std::function<bool()> is_cancelled,
    const int feedback) noexcept; // 0 -> none, 1 -> some, 2 -> verbose

} // namespace perceive::track_ops::detail

namespace perceive::track_ops
{
// Aborts with error message
void validate_tracks_object(const Tracks& tts) noexcept;
} // namespace perceive::track_ops
