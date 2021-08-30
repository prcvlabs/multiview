
#pragma once

#include <opencv2/core/core.hpp>

namespace perceive
{
/**
 * Implements method in Section 3.3 of
 * Pech-Pacheco et al. (2000) - Diatom autofocusing in brightfield microscopy,
 *                              a comparative study
 *
 * Returns the estimated standard deviation of the aboslute value
 * of a narrow (3x3) Laplacian kernel applied to the 'mat'.
 * Converts to greyscale if necessary.
 */
double sharpness_metric(const cv::Mat& mat) noexcept;

} // namespace perceive
