
#pragma once

#include "fowlkes-params.hpp"
#include "json/json.h"

#include "perceive/cost-functions/tracklets/track.hpp"

namespace perceive
{
class Tracks;
struct SceneDescription;

// -------------------------------------------------------------- Fowlkes Result
//
struct FowlkesResult
{
   int w = 0, h = 0, frames = 0, start_frame = 0;

   Vector2 top_left    = {};
   real hist_sz        = 0.0;
   real frame_duration = 0.0;

   vector<Track> tracks;

   FowlkesResult(int w_ = 0, int h_ = 0, int f_ = 0)
       : w(w_)
       , h(h_)
       , frames(f_)
   {}

   bool operator==(const FowlkesResult& o) const noexcept;
   bool operator!=(const FowlkesResult& o) const noexcept;

   size_t memory_usage() const noexcept;

   AABB bounds() const noexcept;

   // ----------------------------------------
   std::string to_string() const noexcept;
   std::string to_json_string() const noexcept;
   Json::Value to_json() const noexcept;

   std::string brief_info() const noexcept;

   Tracks to_tracks() const noexcept;
};

void read(FowlkesResult& data, const Json::Value& node) noexcept(false);
void read(FowlkesResult& data, string_view s) noexcept(false);
void write(const FowlkesResult& data, Json::Value& node) noexcept(false);
void write(const FowlkesResult& data, string& s) noexcept(false);
void save(const FowlkesResult& data, string_view fname) noexcept(false);
void load(FowlkesResult& data, string_view fname) noexcept(false);

// -------------------------------------------------------- make fowlkes results
//
FowlkesResult make_fowlkes_result(
    const int start_frame,
    const int n_frames,
    std::function<const Tracks*(const int frame_no)> get_track_ptr,
    const Vector2& top_left,
    const real hist_sz,
    const real frame_duration) noexcept(false);

} // namespace perceive
