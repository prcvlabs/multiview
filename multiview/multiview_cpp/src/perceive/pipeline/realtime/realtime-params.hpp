
#pragma once

#include "perceive/io/struct-meta.hpp"

namespace perceive::pipeline::realtime
{
// -------------------------------------------------------------- RealtimeParams
// The parameters of `multiview-cli realtime` that `pipeline` does not have.
// Written to params.json under the top-level key "realtime".
//
struct RealtimeParams final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(RealtimeParams)

   virtual ~RealtimeParams() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   // Pacing, in frames per second: 0 is unpaced, and < 0 the scene's fps
   real replay_rate = -1.0;

   string drop_policy           = "auto"s; // "newest", "none", or "auto"
   unsigned n_slots             = 2;       // frames in flight
   unsigned ring_size           = 0;       // decoded frames; 0: n_slots + 4
   unsigned still_sample_period = 15;      // frames between still samples
   unsigned still_max_frames    = 900;     // per sensor; 0: no limit
   string ndjson_path           = "-"s;    // "-" is stdout
   real match_radius            = 0.6;     // metres, provisional ids
   real stats_period_s          = 1.0;     // between `stats` lines
   real max_seconds             = 0.0;     // stop after this long; 0: never
   real missing_timeout_s       = 5.0;     // give up on a frame (drop newest)
   bool publish_provisional     = true;    // write `frame` lines

   static constexpr unsigned k_max_slots = 6;

   // The pacing rate for a scene at `scene_fps`: 0 means unpaced
   real rate(real scene_fps) const noexcept;

   // "newest" when paced, "none" when unpaced, unless set otherwise
   bool drops_newest(real scene_fps) const noexcept;

   unsigned ring_size_or_default() const noexcept;

   // Empty if valid; otherwise what is wrong
   string validate() const noexcept;
};

// Consumes the realtime flag at `argv[i]` and its value, advancing `i` past
// them. False if `argv[i]` is not a realtime flag. Throws std::runtime_error
// on a missing or bad value. This is the `parse_command_line` callback.
bool parse_realtime_arg(RealtimeParams& p,
                        int argc,
                        char** argv,
                        int& i) noexcept(false);

// The realtime flags, for `--help`
string realtime_args_help() noexcept;

} // namespace perceive::pipeline::realtime

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::realtime::RealtimeParams)
} // namespace perceive
