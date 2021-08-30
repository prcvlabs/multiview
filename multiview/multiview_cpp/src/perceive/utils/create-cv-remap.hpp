
#pragma once

#include "perceive/geometry/vector.hpp"
#include "perceive/utils/threads.hpp"
#include "stdinc.hpp"
#include <opencv2/core/core.hpp>

namespace perceive
{
void create_cv_remap(const unsigned w,
                     const unsigned h,
                     const Matrix3r& H_in,
                     std::function<Vector2(const Vector2& x)> f,
                     const Matrix3r& H_out,
                     cv::Mat& mapx,
                     cv::Mat& mapy,
                     const bool feedback = false) noexcept;

void create_cv_remap_threaded(const unsigned w,
                              const unsigned h,
                              const Matrix3r& H_in,
                              std::function<Vector2(const Vector2& x)> f,
                              const Matrix3r& H_out,
                              cv::Mat& mapx,
                              cv::Mat& mapy,
                              ParallelJobSet& job_set,
                              const bool feedback = false) noexcept;

} // namespace perceive
