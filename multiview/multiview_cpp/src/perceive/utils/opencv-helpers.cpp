
#include "opencv-helpers.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"

namespace perceive
{
namespace detail
{
   template<typename T> static MatrixXr to_mat(const cv::Mat& data) noexcept
   {
      MatrixXr o = MatrixXr(data.rows, data.cols);
      for(int row = 0; row < data.rows; ++row)
         for(int col = 0; col < data.cols; ++col)
            o(row, col) = real(data.at<T>(row, col));
      return o;
   }
} // namespace detail

MatrixXr cv_to_matXr(const cv::Mat& data) noexcept
{
   switch(data.type()) {
   case CV_32F: return detail::to_mat<float>(data);
   case CV_64F: return detail::to_mat<double>(data);
   case CV_32S: return detail::to_mat<int32_t>(data);
   default: FATAL(format("type not (yet) supported"));
   }
   return {};
}

VectorXr row_to_vecXr(const cv::Mat& data, const int row) noexcept
{
   Expects(row >= 0 && row < data.rows);
   auto o = VectorXr(data.cols);

   if(data.type() == CV_32F) {
      for(int col = 0; col < data.cols; ++col)
         o(col) = real(data.at<float>(row, col));
   } else if(data.type() == CV_64F) {
      for(int col = 0; col < data.cols; ++col)
         o(col) = data.at<double>(row, col);
   } else {
      Expects(false);
   }

   return o;
}

string str(const cv::Rect roi) noexcept
{
   return format("cv::Rect{{}, {}, {}, {}}",
                 roi.x,
                 roi.y,
                 roi.x + roi.width,
                 roi.y + roi.height);
}

void render_string_cv(cv::Mat& im,
                      const string_view label,
                      const Point2 pos,
                      const uint32_t foreground_colour,
                      const uint32_t background_colour) noexcept
{
   const bool bg_transparent = (background_colour == uint32_t(-1));

   render_string_f(label,
                   foreground_colour,
                   background_colour,
                   [&](int x, int y, uint32_t k) {
                      if((k == foreground_colour) or !bg_transparent) {
                         x += pos.x;
                         y += pos.y;
                         if(x >= 0 and y >= 0 and x < im.cols and y < im.rows) {
                            im.at<cv::Vec3b>(y, x) = rgb_to_vec3b(k);
                         }
                      }
                   });
}

string hexdigest(const vector<cv::Mat>& mats) noexcept
{
   MD5 md5;
   for(const auto& im : mats)
      for(int y = 0; y < im.rows; ++y)
         md5.update(im.ptr(y), unsigned(im.cols) * unsigned(im.elemSize()));
   md5.finalize();
   return md5.hexdigest();
}

} // namespace perceive
