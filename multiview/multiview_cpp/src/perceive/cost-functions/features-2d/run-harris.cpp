
#include "features-2d.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

#include <set>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>

using namespace cv;

namespace perceive
{
// ---------------------------------------------------------------------- Harris
void run_harris(ImageFeatures2d& ss,
                const cv::Mat& grey,
                const ImageFeatures2d::Params& p,
                std::function<bool(void)> is_canceled)
{
   cv::Mat cmat;
   const unsigned w = unsigned(grey.cols), h = unsigned(grey.rows);

   Expects(ss.w != 0);
   Expects(ss.h != 0);
   ss.corners.clear();
   ss.harris_index.clear();

   if(is_canceled()) return;

   // Run harris
   cv::goodFeaturesToTrack(grey,
                           cmat,
                           int(p.max_corners),
                           p.quality_level,
                           p.min_dist,
                           cv::noArray(),
                           int(p.block_size),
                           p.use_harris,
                           p.harris_k);

   if(is_canceled()) return;

   // Go sub-pixel
   if(p.is_subpixel) {
      cv::TermCriteria termcrit(
          cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
      cv::cornerSubPix(grey, cmat, cv::Size(3, 3), cv::Size(-1, -1), termcrit);
   }

   if(is_canceled()) return;

   // Copy out the answer
   unsigned n_corners = unsigned(cmat.rows);

   auto& vec = ss.corners;
   vec.clear();
   vec.resize(n_corners);

   unsigned counter = 0;
   for(int row = 0; row < cmat.rows; ++row) {
      auto& v = vec[counter++];
      v.x     = real(cmat.at<float>(row, 0));
      v.y     = real(cmat.at<float>(row, 1));
      if(v.x < 0 || v.x >= w || v.y < 0 || v.y >= h) {
         --counter;
         // WARN(format("Corner [{} {}] out of range [{} {}]", v.x, v.y, w, h));
      }
   }
   vec.resize(size_t(counter));

   // Set up the spatial index
   ss.harris_index.init(ss.corners);
}

} // namespace perceive
