
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "perceive/geometry/aabb.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
inline cv::Scalar rgb_to_cv(const uint32_t k) noexcept
{
   return cv::Scalar((k >> 0) & 0xff, (k >> 8) & 0xff, (k >> 16) & 0xff);
}

inline cv::Point to_cv_pt(const Vector2& X) noexcept
{
   return cv::Point(int(std::round(X.x)), int(std::round(X.y)));
}

inline bool in_bounds(const cv::Mat& mat, const int x, const int y) noexcept
{
   return unsigned(x) < unsigned(mat.cols) && unsigned(y) < unsigned(mat.rows);
}

inline bool in_bounds(const cv::Mat& mat, const Point2& X) noexcept
{
   return in_bounds(mat, X.x, X.y);
}

void set(cv::Mat& im, int x, int y, uint32_t k) noexcept;
void set(cv::Mat& im, int x, int y, cv::Vec3b k) noexcept;
void blend(cv::Mat& im, int x, int y, uint32_t k, float fg_alpha) noexcept;
void blend(cv::Mat& im, int x, int y, cv::Vec3b k, float fg_alpha) noexcept;

// type() == CV_16SC3 gives: 3 * sizeof(short)
int bits_per_pixel(const cv::Mat& mat) noexcept;

// Split 'dual-im' down the middle, into two images
void hsplit(const cv::Mat& dual_im, cv::Mat& im0, cv::Mat& im1) noexcept(false);

cv::Mat average_image(cv::VideoCapture& cv_video,
                      int start_frame = 0,
                      int n_frames    = -1);

inline AABB cv_im_bounds(const cv::Mat& im) noexcept
{
   return AABB(0, 0, im.cols, im.rows);
}

} // namespace perceive
