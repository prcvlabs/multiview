
#pragma once

#include "perceive/cost-functions/fowlkes/fowlkes-params.hpp"
#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
//
// Grount-truth is there so that we can export training data.
// It cam be an empty fowlkes-result object
//
FowlkesResult run_tracklets_algorithm_1(const pipeline::MovieResults::Params& p,
                                        pipeline::MovieResults* movie) noexcept;

} // namespace perceive
