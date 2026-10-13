
#pragma once

#include "realtime-params.hpp"

namespace perceive::pipeline
{
struct CliArgs;
}

namespace perceive::pipeline::realtime
{
// Runs the per-frame pipeline on the scene's videos replayed at a wall-clock
// rate, as frames arrive: a paced source, a dispatcher, `rp.n_slots` frame
// slots, and a reorder stage that writes one NDJSON line per frame
// (`header`, then `frame` or `dropped` in frame order, `stats` every
// `stats_period_s`, and `end`) to `rp.ndjson_path`. There is no tracker yet.
//
// Writes `params.json` (with `rp` under "realtime") and
// `realtime-stats.json` to `config.outdir`. Stops at the end of the frames,
// after `rp.max_seconds`, or on SIGINT (a second SIGINT exits at once).
// Main thread only. False if the run could not start.
bool run_realtime(const CliArgs& config, const RealtimeParams& rp) noexcept;

} // namespace perceive::pipeline::realtime
