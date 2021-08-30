
#include "ffmpeg.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/process.hpp>

#define This StreamingFFMpegEncoder

namespace perceive::movie
{
namespace bp = boost::process;

// ----------------------------------------------------- call ffmpeg to make mp4
//
int call_ffmpeg_to_make_mp4(const string_view input_fnames,
                            const real frame_duration,
                            const string_view output_fname) noexcept
{
   const string command = format("ffmpeg -y -r {} -i {} -c:v libxvid -qscale:v 2 {} "
                                 "1>/dev/null 2>/dev/null",
                                 (1.0 / frame_duration),
                                 input_fnames,
                                 output_fname);
   return system(command.c_str());
}

// -----------------------------------------------------
//
struct This::Pimpl
{
   bp::pipe pipe;
   bp::child subprocess;
   vector<uint8_t> raw;
   string output_fname;
   int width  = 0;
   int height = 0;
   real frame_rate; //

   ~Pimpl() { close(); }

   void init(const string_view output_fname, int w, int h, real frame_rate)
   {
      this->output_fname = output_fname;
      this->width        = w;
      this->height       = h;
      this->frame_rate   = frame_rate;

      raw.resize(size_t(width * 3)); // rgb

      auto find_ffmpeg_exe = [&]() { // attempt to find the ffmpeg executable
         const char* exe = "ffmpeg";
         const auto path = bp::search_path(exe);
         if(path.empty())
            throw std::runtime_error(
                format("failed to find executable '{}' on path, aborting", exe));
         return path;
      };
      const auto exe_path = find_ffmpeg_exe();

      // bp::std_out > stdout,
      subprocess = bp::child(exe_path,
                             "-hide_banner", // really
                             "-y",           // allow overwrite
                             "-f",           // input format
                             "rawvideo",
                             "-pix_fmt", // input pixel format
                             "rgb24",
                             "-video_size", // input format
                             format("{}x{}", width, height),
                             "-r", // framerate
                             str(frame_rate),
                             "-i", // input
                             "-",
                             "-c:v", // video codec
                             "libxvid",
                             "-qscale:v", // quality
                             "2",
                             output_fname.data(),
                             bp::std_out > bp::null,
                             bp::std_err > bp::null,
                             bp::std_in < pipe);
   }

   void push_frame(const cv::Mat& im)
   {
      if((im.rows != height) || (im.cols != width))
         throw std::runtime_error(format("frame shape mismatch"));

      if(im.type() == CV_8UC3) {
         for(auto y = 0; y < height; ++y) {
            const uint8_t* row = im.ptr(y); // BGR format
            uint8_t* out       = &raw[0];
            for(auto x = 0; x < width; ++x) {
               uint8_t b = *row++;
               uint8_t g = *row++;
               uint8_t r = *row++;
               *out++    = r;
               *out++    = g;
               *out++    = b;
            }
            Expects(out == &raw[0] + raw.size());
            pipe.write(reinterpret_cast<char*>(&raw[0]), int(raw.size()));
         }
      } else if(im.type() == CV_8UC1 || im.type() == CV_8U) {
         for(auto y = 0; y < height; ++y) {
            const uint8_t* row = im.ptr(y); // gray format
            uint8_t* out       = &raw[0];
            for(auto x = 0; x < width; ++x) {
               uint8_t g = *row++;
               *out++    = g;
               *out++    = g;
               *out++    = g;
            }
            Expects(out == &raw[0] + raw.size());
            pipe.write(reinterpret_cast<char*>(&raw[0]), int(raw.size()));
         }
      } else {
         FATAL(format("input movie was in unsupport color space: cv::Mat.type() == {}",
                      im.type()));
      }
   }

   int close()
   {
      if(pipe.is_open()) {
         pipe.close();
         std::error_code ec;
         subprocess.wait(ec);
         if(ec) {
            throw std::runtime_error(
                format("error waiting for ffmpeg to end: {}", ec.message()));
         }
      }
      return subprocess.exit_code();
   }
};

This::This()
    : pimpl_(new Pimpl)
{}
This::~This() = default;
void This::push_frame(const cv::Mat& frame) { pimpl_->push_frame(frame); }
int This::close() { return pimpl_->close(); }

StreamingFFMpegEncoder This::create(const string_view output_fname,
                                    const int width,
                                    const int height,
                                    const real frame_rate)
{
   StreamingFFMpegEncoder o;
   o.pimpl_->init(output_fname, width, height, frame_rate);
   return o;
}

} // namespace perceive::movie
