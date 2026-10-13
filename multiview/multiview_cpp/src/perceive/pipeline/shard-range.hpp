
#pragma once

#include "stdinc.hpp"

namespace perceive
{
struct LocalizationDataEnvelope;
}

namespace perceive::pipeline
{
// Checks that the frames [start, start + n) of a movie with `total` frames
// form a shard that the tracker can process on its own: `start` and the end
// frame are multiples of `k` (max-frames-per-tracklet), except that the end
// may be the end of the movie, and there are at least 2 frames. If `env` is
// not null, then the range must also lie inside the envelope's frames.
// Returns a message for the log, or nothing if the range is fine.
std::optional<string>
shard_alignment_error(const int start,
                      const int n,
                      const int total,
                      const int k,
                      const LocalizationDataEnvelope* env) noexcept;

} // namespace perceive::pipeline
