
#include "stdinc.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace perceive
{
/**
 * Implements method in Section 3.3 of
 * Pech-Pacheco et al. (2000) - Diatom autofocusing in brightfield microscopy,
 *                              a comparative study.
 *
 * Returns the estimated standard deviation of the aboslute value
 * of a narrow (3x3) Laplacian kernel applied to the 'mat'.
 * Converts to greyscale if necessary.
 */
double sharpness_metric(const cv::Mat& mat) noexcept
{
   // If the image isn't greyscale, then we need to change it
   const cv::Mat* im = &mat;
   cv::Mat grey_data;
   if(mat.channels() > 1) {
      Expects(mat.channels() == 3);
      cv::cvtColor(mat, grey_data, cv::COLOR_BGR2GRAY);
      im = &grey_data;
   }
   const cv::Mat& grey = *im;

   // Apply a narrow Laplacian operator
   cv::Mat laplace;
   cv::Laplacian(grey, laplace, CV_16S);

   // Find the average of the absolute values
   double sum = 0.0;
   for(auto y = 0; y < laplace.rows; ++y) {
      const short* row = reinterpret_cast<short*>(laplace.ptr(y));
      for(auto x = 0; x < laplace.cols; ++x) sum += abs(row[x]);
   }
   double average = sum / real(laplace.rows * laplace.cols);

   // Find the variance estimtate
   double sum_sq = 0.0;
   for(auto y = 0; y < laplace.rows; ++y) {
      const short* row = reinterpret_cast<short*>(laplace.ptr(y));
      for(auto x = 0; x < laplace.cols; ++x)
         sum_sq += square(abs(row[x]) - average);
   }
   double est_var = sum_sq / real(laplace.rows * laplace.cols - 1);

   return sqrt(est_var); // DONE! (Higher is sharper)
}

} // namespace perceive
