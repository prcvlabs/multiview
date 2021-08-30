
#pragma once

#include "fowlkes-params.hpp"
#include "fowlkes-result.hpp"

namespace perceive
{
// --------------------------------------------------------- filter short tracks
//
FowlkesResult filter_short_tracks(const fowlkes::Params& params,
                                  const FowlkesResult& result) noexcept;

// --------------------------------------------------------------- smooth tracks
//
FowlkesResult smooth_tracks(const fowlkes::Params& params,
                            const FowlkesResult& result) noexcept;

// --------------------------------------------------------------- rebase tracks
//
FowlkesResult rebase_tracks(const FowlkesResult& result) noexcept;

} // namespace perceive
