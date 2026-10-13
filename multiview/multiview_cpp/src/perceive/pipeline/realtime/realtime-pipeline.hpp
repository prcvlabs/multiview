
#pragma once

#include "realtime-params.hpp"

namespace perceive::pipeline
{
struct CliArgs;
}

namespace perceive::pipeline::realtime
{
// Runs the pipeline on the scene's videos replayed at a wall-clock rate, as
// frames arrive: a paced source, a dispatcher, `rp.n_slots` frame slots, a
// reorder stage that writes one NDJSON line per frame (`frame` or `dropped`,
// in frame order) and hands the frames to the online tracker, which writes
// `tracks`, `smoothed`, `track_end` and `window_error` lines. The stream
// starts with `header` and ends with `end`; `stats` comes every
// `stats_period_s`. Lines go to `rp.ndjson_path`.
//
// At the end, writes `pipeline-output.json` as `pipeline` does (and, before
// smoothing, `pipeline-output-presmooth.json`), scores it when
// `config.run_pipeline_testcase` is set, and writes `params.json` (with `rp`
// under "realtime") and `realtime-stats.json` to `config.outdir`. Stops at
// the end of the frames, after `rp.max_seconds`, on SIGINT (a second SIGINT
// exits at once), or when a tracking window fails. Main thread only. False
// if the run could not start, the tracker failed, or the tracks could not
// be written.
bool run_realtime(const CliArgs& config, const RealtimeParams& rp) noexcept;

// Writes `k movie-frame` for every scene frame `k` of the scene in `config`:
// the frame that the paced source decodes for `k`, before each camera's
// offset. With `--target-fps` below the videos' rate, scene frames skip
// movie frames, and pose replay files (named by scene frame) recorded at the
// videos' rate must be renamed (`bin/pose-replay-remap.py`). False on error.
bool print_frame_mapping(const CliArgs& config, std::ostream& out) noexcept;

} // namespace perceive::pipeline::realtime
