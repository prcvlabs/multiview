
#include <algorithm>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"
#include "testcases/testcase-data.hpp"

#include <opencv2/videoio.hpp>

namespace perceive
{
// ------------------------------------------------------------ testcase video
// The first video of testcase 007, if it has been fetched
// (build/fetch-testcases.sh 7). The videos have a keyframe every 250 frames.

static string testcase_video_filename() noexcept
{
   return format("{}/testcases/007_site03_sparse_tracks/C0001020_v1.mp4",
                 test::testcase_data_dir());
}

static string frame_digest(const cv::Mat& im) noexcept
{
   if(im.empty()) return ""s;
   Expects(im.isContinuous());
   return md5(im.data, im.total() * im.elemSize());
}

// ---------------------------------------------------------- read from start
// Plain decoding: element `k` is the picture of frame `k`

static vector<string> read_from_start(const string& fname,
                                      const int n_frames) noexcept
{
   vector<string> out;
   cv::VideoCapture video(fname);
   cv::Mat im;
   for(int f = 0; f < n_frames; ++f) {
      video >> im;
      out.push_back(frame_digest(im));
   }
   return out;
}

// ---------------------------------------------------- read frames by seeking
// What SceneDescription does for a frame that is not the next one: seek,
// then retrieve

static vector<string> read_by_seeking(const string& fname,
                                      const int frame0,
                                      const int n_frames) noexcept
{
   vector<string> out;
   cv::VideoCapture video(fname);
   cv::Mat im;
   for(int f = frame0; f < frame0 + n_frames; ++f) {
      video.set(cv::CAP_PROP_POS_FRAMES, f);
      video.retrieve(im);
      out.push_back(frame_digest(im));
   }
   return out;
}

// ------------------------------------------------- read frames sequentially
// What SceneDescription does when the next frame is requested: one seek,
// then `>>` for every frame after it

static vector<string> read_sequentially(const string& fname,
                                        const int frame0,
                                        const int n_frames) noexcept
{
   vector<string> out;
   cv::VideoCapture video(fname);
   cv::Mat im;
   video.set(cv::CAP_PROP_POS_FRAMES, frame0);
   video.retrieve(im);
   out.push_back(frame_digest(im));
   for(int f = frame0 + 1; f < frame0 + n_frames; ++f) {
      video >> im;
      out.push_back(frame_digest(im));
   }
   return out;
}

// ----------------------------------------------------------------- testcase

CATCH_TEST_CASE("VideoDecode", "[video_decode]")
{
   const auto fname = testcase_video_filename();
   if(!is_regular_file(fname)) {
      WARN(format("testcase video '{}' not found, skipping", fname));
      return;
   }

   // Sequential reads must give the same pictures as seeking to each frame,
   // near the start of the video and across a keyframe boundary
   CATCH_SECTION("sequential-equals-seek")
   {
      for(const auto [frame0, n_frames] : {std::pair{1, 24}, {240, 24}}) {
         const auto seeked     = read_by_seeking(fname, frame0, n_frames);
         const auto sequential = read_sequentially(fname, frame0, n_frames);
         CATCH_REQUIRE(seeked.size() == size_t(n_frames));
         CATCH_REQUIRE(sequential.size() == size_t(n_frames));
         for(int i = 0; i < n_frames; ++i) {
            CATCH_INFO(format("frame {}", frame0 + i));
            CATCH_REQUIRE(!seeked[size_t(i)].empty());
            CATCH_REQUIRE(seeked[size_t(i)] == sequential[size_t(i)]);
         }
         // Consecutive frames of a video of moving people differ
         CATCH_REQUIRE(std::adjacent_find(cbegin(seeked), cend(seeked))
                       == cend(seeked));
      }
   }

   // Seeking to frame `n` (n >= 1) and retrieving yields the picture of
   // frame `n - 1`, and seeking to frame 0 yields frame 0: OpenCV's ffmpeg
   // backend decodes up to the frame before the target and leaves the target
   // for the next grab. The pipeline has always addressed frames this way
   // (scene frames 0 and 1 are the same picture) and the sequential path
   // reproduces it; this pins the behaviour in case a future OpenCV changes
   CATCH_SECTION("seek-then-retrieve-is-the-previous-frame")
   {
      const auto plain  = read_from_start(fname, 8);
      const auto seeked = read_by_seeking(fname, 0, 8);
      CATCH_REQUIRE(seeked[0] == plain[0]);
      for(int n = 1; n < 8; ++n) {
         CATCH_INFO(format("frame {}", n));
         CATCH_REQUIRE(!plain[size_t(n)].empty());
         CATCH_REQUIRE(seeked[size_t(n)] == plain[size_t(n - 1)]);
      }
   }
}

} // namespace perceive
