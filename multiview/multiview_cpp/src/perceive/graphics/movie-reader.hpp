
#pragma once

namespace cv
{
class Mat;
class VideoCapture;
} // namespace cv

namespace perceive
{
struct MovieReader
{
 private:
   int n_frames_ = 0;
   int frame_no_ = 0;
   unique_ptr<cv::VideoCapture> video_;
   unique_ptr<cv::Mat> frame_;

 public:
   ~MovieReader();

   bool open(string_view fname) noexcept;

   int frame_no() const noexcept;
   int n_frames() const noexcept;

   bool seek(int frame) noexcept;

   const cv::Mat& current_frame();
   const cv::Mat& prev_frame();
   const cv::Mat& next_frame();
};

} // namespace perceive
