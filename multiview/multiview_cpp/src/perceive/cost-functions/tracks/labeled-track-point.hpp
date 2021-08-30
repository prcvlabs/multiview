
#pragma once

#include "perceive/cost-functions/tracklets/track.hpp"
#include "perceive/io/json-io.hpp"

namespace perceive
{
struct LabeledTrackPoint
{
   int track_id = -1;
   TrackPoint tp{};

   bool operator==(const LabeledTrackPoint& o) const noexcept
   {
      return track_id == o.track_id && tp == o.tp;
   }
   bool operator!=(const LabeledTrackPoint& o) const noexcept
   {
      return !(*this == o);
   }

   Json::Value to_json() const noexcept;
   bool load_json(const Json::Value&) noexcept(false);
};
vector<LabeledTrackPoint> tracks_to_trackpoints(const vector<Track>&) noexcept;

bool check_trackpoints_as_tracks(const vector<LabeledTrackPoint>& in) noexcept;
vector<Track> trackpoints_to_tracks(const vector<LabeledTrackPoint>&) noexcept;

vector<LabeledTrackPoint>
extract_labeled_trackpoints(const vector<Track>& tracks,
                            const int frame_no) noexcept;

} // namespace perceive
