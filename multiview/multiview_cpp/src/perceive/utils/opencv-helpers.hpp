
#pragma once

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "md5.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
namespace detail
{
   template<typename EigenType>
   inline cv::Mat to_cv_mat_32f(const EigenType& e) noexcept
   {
      int n_rows = int(e.rows());
      int n_cols = int(e.cols());
      cv::Mat m(n_rows, n_cols, CV_32F);
      for(int i = 0; i < n_rows; ++i)
         for(int j = 0; j < n_cols; ++j) m.at<float>(i, j) = float(e(i, j));
      return m;
   }
} // namespace detail

inline cv::Mat to_cv_mat_32f(const Eigen::Matrix<real, 3, 3>& e) noexcept
{
   return detail::to_cv_mat_32f(e);
}

template<typename T>
inline cv::Mat to_cv_mat_32f(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& e) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return detail::to_cv_mat_32f(e);
}

template<typename T>
inline cv::Mat
to_cv_mat_32f(const Eigen::Matrix<T, Eigen::Dynamic, 1>& e) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return detail::to_cv_mat_32f(e);
}

inline cv::Mat unpack_vector(const real* d, uint32_t len) noexcept
{
   cv::Mat m(int(len), 1, CV_32F);
   for(int i = 0; i < int(len); ++i) m.at<float>(i, 0) = float(d[i]);
   return m;
}

MatrixXr cv_to_matXr(const cv::Mat& data) noexcept;

VectorXr row_to_vecXr(const cv::Mat& data, const int row) noexcept;

inline Vector3 to_vec3(const cv::Vec3d X) noexcept
{
   return Vector3(X(0), X(1), X(2));
}

inline cv::Rect to_cv_rect(const AABBi& aabb) noexcept
{
   return cv::Rect(aabb.left, aabb.top, aabb.width(), aabb.height());
}

inline AABBi image_bounds(const cv::Mat& im) noexcept
{
   return AABBi(0, 0, im.cols, im.rows);
}

string str(const cv::Rect roi) noexcept;

// The default background color is transparent
void render_string_cv(cv::Mat& im,
                      const string_view label,
                      const Point2 pos,
                      const uint32_t foreground_colour,
                      const uint32_t background_colour = uint32_t(-1)) noexcept;

string hexdigest(const vector<cv::Mat>& mats) noexcept;

} // namespace perceive
