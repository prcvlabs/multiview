
#include "stdinc.hpp"

#include "cv-helpers.hpp"

#include "perceive/graphics/image-container.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace perceive
{
int bits_per_pixel(const cv::Mat& mat) noexcept { return int(mat.elemSize()); }

void hsplit(const cv::Mat& dual_im, cv::Mat& im0, cv::Mat& im1) noexcept(false)
{
   if(dual_im.cols % 2 != 0)
      throw std::runtime_error("excepted even number of image columns");

   auto w = dual_im.cols / 2;
   auto h = dual_im.rows;

   im0.create(cv::Size(w, h), dual_im.type());
   im1.create(cv::Size(w, h), dual_im.type());
   const auto bpp = bits_per_pixel(dual_im);
   for(auto y = 0; y < h; ++y) {
      const uint8_t* src = dual_im.ptr(y);
      uint8_t* dst1      = im0.ptr(y);
      uint8_t* dst2      = im1.ptr(y);
      memcpy(dst1, src, size_t(bpp * w));
      memcpy(dst2, src + bpp * w, size_t(bpp * w));
   }
}

cv::Mat average_image(cv::VideoCapture& cv_video, int start_frame, int n_frames)
{
   Vec3fImage im;
   bool first  = true;
   int counter = 0;

   auto now = tick();
   cv_video.set(cv::CAP_PROP_POS_FRAMES, start_frame);

   while(true) {
      cv::Mat frame;
      cv_video >> frame;
      if(frame.empty()) break;
      if(first) {
         im    = cv_to_LAB_vec3f_im(frame);
         first = false;
      } else {
         im += cv_to_LAB_vec3f_im(frame);
      }
      if(counter > 0 and counter % 40 == 0)
         cout << format(" + frame {}, {}s", counter, tock(now)) << endl;
      if((counter >= 0) and (counter == n_frames)) break;
      counter++;
   }

   std::for_each(im.begin(), im.end(), [&](auto& x) { x /= float(counter); });

   return argb_to_cv(LAB_vec3f_im_to_argb(im));
}

// ------------------------------------------------------------------------- set

void set(cv::Mat& im, int x, int y, uint32_t k) noexcept
{
   cv::Vec3b k0 = rgb_to_vec3b(k);
   set(im, x, y, k0);
}

void set(cv::Mat& im, int x, int y, cv::Vec3b k) noexcept
{
   if(x < 0 or y < 0) return;
   if(x >= im.cols or y >= im.rows) return;

   if(im.type() == CV_8UC3) {
      im.at<cv::Vec3b>(y, x) = k;
   } else if(im.type() == CV_8UC1 || im.type() == CV_8U) {
      im.at<uint8_t>(y, x)
          = uint8_t((real(k[0]) + real(k[1]) + real(k[2])) / 3.0);
   } else {
      FATAL("image type not supported");
   }
}

void blend(cv::Mat& im, int x, int y, uint32_t k, float fg_alpha) noexcept
{
   cv::Vec3b k0 = rgb_to_vec3b(k);
   blend(im, x, y, k0, fg_alpha);
}

void blend(cv::Mat& im, int x, int y, cv::Vec3b k, float fg_alpha) noexcept
{
   if(!in_bounds(im, x, y)) return;
   if(im.type() == CV_8UC3) {
      im.at<cv::Vec3b>(y, x) = blend(k, im.at<cv::Vec3b>(y, x), fg_alpha);
   } else if(im.type() == CV_8UC1 || im.type() == CV_8U) {
      FATAL("image type not supported");
   } else {
      FATAL("image type not supported");
   }
}

} // namespace perceive
