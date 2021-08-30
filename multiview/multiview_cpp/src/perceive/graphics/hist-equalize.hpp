
#pragma once

#include <opencv2/core/mat.hpp>

namespace perceive
{
void hist_equalize(const cv::Mat& in, cv::Mat& out);

} // namespace perceive
