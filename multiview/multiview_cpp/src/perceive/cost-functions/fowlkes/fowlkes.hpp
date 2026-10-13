
#pragma once

#include "fowlkes-params.hpp"
#include "fowlkes-result.hpp"

namespace perceive
{
struct KzFilter;

// --------------------------------------------------------- filter short tracks
//
FowlkesResult filter_short_tracks(const fowlkes::Params& params,
                                  const FowlkesResult& result) noexcept;

// ---------------------------------------------------------------- smooth track
// Smooths one track's positions, gazes and pose scores with `filter`
//
void smooth_track(const KzFilter& filter, Track& tt) noexcept;

// --------------------------------------------------------------- smooth tracks
//
FowlkesResult smooth_tracks(const fowlkes::Params& params,
                            const FowlkesResult& result) noexcept;

// --------------------------------------------------------------- rebase tracks
//
FowlkesResult rebase_tracks(const FowlkesResult& result) noexcept;

} // namespace perceive
