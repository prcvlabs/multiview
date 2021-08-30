
#include "movie-reader.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This MovieReader

namespace perceive
{
This::~This() = default;

bool This::open(string_view fname) noexcept
{
   video_ = make_unique<cv::VideoCapture>(fname.data());
   frame_ = make_unique<cv::Mat>();
   if(!video_ || !video_->isOpened()) {
      video_.release();
      frame_no_ = n_frames_ = 0;
      return false;
   }
   frame_no_ = -1;
   n_frames_ = int(video_->get(cv::CAP_PROP_FRAME_COUNT));
   seek(0);
   return true;
}

int This::frame_no() const noexcept { return frame_no_; }

int This::n_frames() const noexcept { return n_frames_; }

bool This::seek(int frame) noexcept
{
   Expects(video_);
   if(frame >= 0 && frame < n_frames_ && frame != frame_no_) {
      frame_no_ = frame;
      video_->set(cv::CAP_PROP_POS_FRAMES, frame_no_ + 1);
      return true;
   }
   return false; // frame-no didn't change
}

const cv::Mat& This::current_frame()
{
   Expects(video_);
   Expects(frame_);
   video_->retrieve(*frame_);
   return *frame_;
}

const cv::Mat& This::prev_frame()
{
   Expects(video_);
   seek(frame_no_ - 1);
   return current_frame();
}

const cv::Mat& This::next_frame()
{
   Expects(video_);
   seek(frame_no_ + 1);
   return current_frame();
}

} // namespace perceive

#undef This
