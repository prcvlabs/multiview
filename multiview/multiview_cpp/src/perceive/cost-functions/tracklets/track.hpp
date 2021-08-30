
#pragma once

#include "track-point.hpp"

#include "perceive/geometry/line-1d.hpp"

namespace perceive
{
struct TrackFalsePositiveInfo
{
   float prob_fp_p       = fNAN;
   float dist_p          = fNAN;
   float lab_p           = fNAN;
   float duration_p      = fNAN;
   float score           = fNAN;
   bool is_true_positive = false;
};

struct Track
{
   enum Label : int { UNLABELLED = 0, CUSTOMER, EMPLOYEE };

   int id = -1; // Valid tracks have an id >= 0
   vector<TrackPoint> path;
   float cost   = 0.0f;
   float height = 0.0f;
   Label label  = UNLABELLED;

   TrackFalsePositiveInfo fp_info;

   bool operator==(const Track& o) const noexcept;
   bool operator!=(const Track& o) const noexcept;

   size_t memory_usage() const noexcept;

   size_t size() const noexcept { return path.size(); }
   bool empty() const noexcept { return size() == 0; }
   bool is_valid() const noexcept; // WARNING, O(n) in size of path

   // NOTE: right is exclusive, [path.front.t, path.back.t + 1]
   Line1Di path_time_range() const noexcept; // only valid of 'is_valid'

   // EVERY time-point in 'tt' is before path. Expects(is_valid())
   // Always FALSE if either track is empty
   bool before(const Track& tt) const noexcept;
   bool after(const Track& tt) const noexcept; // ibid, but after

   // ----------------------------------------
   std::string brief_info() const noexcept;
   std::string to_string() const noexcept;
   std::string to_json_string() const noexcept;
   Json::Value to_json() const noexcept;
   void read_with_defaults(const Json::Value& o,
                           const Track* defaults = nullptr) noexcept;

   friend string str(const Track& o) { return o.to_string(); }
   friend string str(Label o);
};

Track::Label to_track_label(const string_view s) noexcept(false);

string quick_view(const vector<Track>& tts) noexcept;

META_READ_WRITE_LOAD_SAVE(Track)

} // namespace perceive
