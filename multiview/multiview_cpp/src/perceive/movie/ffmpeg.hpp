
#pragma once

#include <opencv2/core/core.hpp>

namespace perceive::movie
{
// ------------------------------------------------- call ffmpeg to make a movie
// All frames must exist on disk
int call_ffmpeg_to_make_mp4(const string_view input_fnames,
                            const real frame_duration,
                            const string_view output_fname) noexcept;

// ------------------------------------------------- call ffmpeg to make a movie
// Streaming encoder... encodes on the fly
//
// EXAMPLE
//
// auto video = make_unique<cv::VideoCapture>(movie_filename);
// cv::Mat im;
// *video >> im;
// if(im.empty())
//    FATAL(format("failed to read a single frame from video file, aborting"));
// const int width  = im.cols;
// const int height = im.rows;
// auto encoder     = movie::StreamingFFMpegEncoder::create(
//                               output_fname, width, height, 15.0);
//
// { // ---- Process the movie
//    int frame_no = 0;
//    while(true) {
//       encoder.push_frame(im);
//       *video >> im;
//       if(im.empty()) break;
//    }
// }
//
// return encoder.close();
//
struct StreamingFFMpegEncoder
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

   StreamingFFMpegEncoder();

 public:
   StreamingFFMpegEncoder(const StreamingFFMpegEncoder&) = delete;
   StreamingFFMpegEncoder(StreamingFFMpegEncoder&&)      = default;
   ~StreamingFFMpegEncoder();
   StreamingFFMpegEncoder& operator=(const StreamingFFMpegEncoder&) = delete;
   StreamingFFMpegEncoder& operator=(StreamingFFMpegEncoder&&) = default;

   static StreamingFFMpegEncoder create(const string_view output_fname,
                                        const int width,
                                        const int height,
                                        const real frame_rate);

   void push_frame(const cv::Mat& frame);
   int close(); // waits for ffmpeg to finish encoding, returns exit-code
};

} // namespace perceive::movie
