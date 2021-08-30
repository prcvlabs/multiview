
#include "hist-equalize.hpp"
#include "stdinc.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace perceive
{
void hist_equalize(const cv::Mat& in, cv::Mat& out)
{
   Expects(in.channels() == 3 || in.channels() == 1);

   if(in.channels() == 1) {
      cv::equalizeHist(in, out);

   } else if(in.channels() == 3) {
      cv::Mat ycrcb;
      cv::cvtColor(in, ycrcb, cv::COLOR_BGR2YCrCb);
      vector<cv::Mat> channels;
      cv::split(ycrcb, channels);
      cv::equalizeHist(channels[0], channels[0]);
      cv::merge(channels, ycrcb);
      cv::cvtColor(ycrcb, out, cv::COLOR_YCrCb2BGR);

   } else {
      FATAL("kBAM!");
   }
}

} // namespace perceive
