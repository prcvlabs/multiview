
#include "realtime-params.hpp"

#include "perceive/utils/cli-utils.hpp"

namespace perceive::pipeline::realtime
{
// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& RealtimeParams::meta_data() const noexcept
{
#define ThisParams RealtimeParams
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, REAL, replay_rate, true));
      m.push_back(MAKE_META(ThisParams, STRING, drop_policy, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, n_slots, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, ring_size, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, still_sample_period, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, still_max_frames, true));
      m.push_back(MAKE_META(ThisParams, STRING, ndjson_path, false));
      m.push_back(MAKE_META(ThisParams, REAL, match_radius, true));
      m.push_back(MAKE_META(ThisParams, REAL, stats_period_s, false));
      m.push_back(MAKE_META(ThisParams, REAL, max_seconds, true));
      m.push_back(MAKE_META(ThisParams, REAL, missing_timeout_s, true));
      m.push_back(MAKE_META(ThisParams, BOOL, publish_provisional, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ------------------------------------------------------------------------ rate
//
real RealtimeParams::rate(real scene_fps) const noexcept
{
   return (replay_rate < 0.0) ? scene_fps : replay_rate;
}

// ---------------------------------------------------------------- drops-newest
//
bool RealtimeParams::drops_newest(real scene_fps) const noexcept
{
   if(rate(scene_fps) <= 0.0) return false; // unpaced never drops
   return drop_policy != "none"s;
}

// -------------------------------------------------------- ring-size-or-default
//
unsigned RealtimeParams::ring_size_or_default() const noexcept
{
   return (ring_size > 0) ? ring_size : n_slots + 4;
}

// -------------------------------------------------------------------- validate
//
string RealtimeParams::validate() const noexcept
{
   vector<string> errors;
   if(n_slots < 1 or n_slots > k_max_slots)
      errors.push_back(format("--slots must be in [1, {}]", k_max_slots));
   if(drop_policy != "auto"s and drop_policy != "newest"s
      and drop_policy != "none"s)
      errors.push_back(
          format("--drop must be 'newest' or 'none', not '{}'", drop_policy));
   if(!std::isfinite(replay_rate))
      errors.push_back("--replay-rate must be a number"s);
   if(ndjson_path.empty()) errors.push_back("--out must not be empty"s);
   if(!(match_radius > 0.0)) errors.push_back("--match-radius must be > 0"s);
   if(!(stats_period_s > 0.0)) errors.push_back("--stats-period must be > 0"s);
   if(!(max_seconds >= 0.0)) errors.push_back("--max-seconds must be >= 0"s);
   if(!(missing_timeout_s > 0.0))
      errors.push_back("--missing-timeout must be > 0"s);
   return implode(cbegin(errors), cend(errors), "; ");
}

// ---------------------------------------------------------- parse-realtime-arg
//
bool parse_realtime_arg(RealtimeParams& p,
                        int argc,
                        char** argv,
                        int& i) noexcept(false)
{
   const string arg = argv[i];
   if(arg == "--replay-rate"s) {
      p.replay_rate = cli::safe_arg_real(argc, argv, i);
   } else if(arg == "--drop"s) {
      p.drop_policy = cli::safe_arg_str(argc, argv, i);
   } else if(arg == "--slots"s) {
      p.n_slots = unsigned(std::max(0, cli::safe_arg_int(argc, argv, i)));
   } else if(arg == "--ring"s) {
      p.ring_size = unsigned(std::max(0, cli::safe_arg_int(argc, argv, i)));
   } else if(arg == "--still-every"s) {
      p.still_sample_period
          = unsigned(std::max(1, cli::safe_arg_int(argc, argv, i)));
   } else if(arg == "--still-max"s) {
      p.still_max_frames
          = unsigned(std::max(0, cli::safe_arg_int(argc, argv, i)));
   } else if(arg == "--out"s or arg == "--ndjson"s) {
      p.ndjson_path = cli::safe_arg_str(argc, argv, i);
   } else if(arg == "--match-radius"s) {
      p.match_radius = cli::safe_arg_real(argc, argv, i);
   } else if(arg == "--stats-period"s) {
      p.stats_period_s = cli::safe_arg_real(argc, argv, i);
   } else if(arg == "--max-seconds"s) {
      p.max_seconds = cli::safe_arg_real(argc, argv, i);
   } else if(arg == "--missing-timeout"s) {
      p.missing_timeout_s = cli::safe_arg_real(argc, argv, i);
   } else if(arg == "--no-provisional"s) {
      p.publish_provisional = false;
   } else {
      return false;
   }
   return true;
}

// ---------------------------------------------------------- realtime-args-help
//
string realtime_args_help() noexcept
{
   const RealtimeParams o;
   return format(R"V0G0N(
   Real-time options:

      --replay-rate <fps>    Replay the videos at this rate. 0 is as fast as
                             frames are taken (no drops). Default: scene fps.
      --drop <newest|none>   When every slot is busy: drop the frames waiting
                             and take the newest, or wait. Default: newest
                             when paced, none when unpaced.
      --slots <n>            Frames in flight, 1..{}. Default: {}.
      --ring <n>             Decoded frames held. Default: slots + 4.
      --still-every <n>      Add every n-th frame to the still. Default: {}.
      --still-max <n>        Stop the still at n frames (0: never). Default: {}.
      --out <file|->         NDJSON output; '-' is stdout. Default: '{}'.
      --match-radius <m>     Provisional id radius, metres. Default: {}.
      --stats-period <s>     Seconds between 'stats' lines. Default: {}.
      --max-seconds <s>      Stop after this long (0: never). Default: {}.
      --missing-timeout <s>  Give up on a frame after this long (drop newest
                             only). Default: {}.
      --no-provisional       Do not write 'frame' lines.
)V0G0N",
                 RealtimeParams::k_max_slots,
                 o.n_slots,
                 o.still_sample_period,
                 o.still_max_frames,
                 o.ndjson_path,
                 o.match_radius,
                 o.stats_period_s,
                 o.max_seconds,
                 o.missing_timeout_s);
}

} // namespace perceive::pipeline::realtime
