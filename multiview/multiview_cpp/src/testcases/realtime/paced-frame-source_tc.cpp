
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/realtime/paced-frame-source.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

#include <atomic>
#include <thread>

#include <opencv2/videoio.hpp>

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;
using time_point = std::chrono::steady_clock::time_point;

// ------------------------------------------------------------------- FakeClock
// Time passes only when someone sleeps, or a fake camera decodes.
//
struct FakeClock
{
   std::atomic<int64_t> ns = int64_t(1000) * 1000 * 1000 * 1000;

   time_point now() const noexcept
   {
      return time_point{} + std::chrono::nanoseconds(ns.load());
   }

   void advance(std::chrono::nanoseconds dt) noexcept { ns += dt.count(); }

   void sleep_until(time_point t) noexcept
   {
      const int64_t t_ns = (t - time_point{}).count();
      int64_t cur        = ns.load();
      while(cur < t_ns and !ns.compare_exchange_weak(cur, t_ns)) {}
   }
};

// ------------------------------------------------------------ FakeCameraReader
// The picture of frame `n` is a 4x6 image of colour (n % 256, camera, 0).
//
class FakeCameraReader final : public CameraReader
{
 private:
   int cam_          = 0;
   int fail_frame_   = -1;
   FakeClock* clock_ = nullptr;
   std::chrono::nanoseconds cost_{0};

 public:
   FakeCameraReader(int cam,
                    int fail_frame                = -1,
                    FakeClock* clock              = nullptr,
                    std::chrono::nanoseconds cost = 0ns)
       : cam_(cam)
       , fail_frame_(fail_frame)
       , clock_(clock)
       , cost_(cost)
   {}

   bool read(int frame_no, cv::Mat& out) noexcept override
   {
      if(clock_ != nullptr) clock_->advance(cost_);
      if(frame_no == fail_frame_) return false;
      out = cv::Mat(4, 6, CV_8UC3, cv::Scalar(frame_no % 256, cam_, 0));
      return true;
   }
};

static vector<unique_ptr<CameraReader>>
make_readers(int n_cameras, int fail_cam = -1, int fail_frame = -1)
{
   vector<unique_ptr<CameraReader>> readers;
   for(int i = 0; i < n_cameras; ++i)
      readers.push_back(
          make_unique<FakeCameraReader>(i, (i == fail_cam) ? fail_frame : -1));
   return readers;
}

static PacedFrameSource::Params
make_params(int n_frames, real rate_fps, FakeClock* clock)
{
   PacedFrameSource::Params p;
   p.start_frame = 0;
   p.end_frame   = n_frames;
   p.scene_fps   = 15.0;
   p.timestamp0  = Timestamp("2020-02-02T10:00:00.000000");
   p.rate_fps    = rate_fps;
   p.ring_size   = unsigned(n_frames + 1);
   if(clock != nullptr) {
      p.now         = [clock]() { return clock->now(); };
      p.sleep_until = [clock](time_point t) { clock->sleep_until(t); };
   }
   return p;
}

// Everything the source delivers, until the end of the stream
static vector<shared_ptr<const SourceFrame>> drain(PacedFrameSource& source)
{
   vector<shared_ptr<const SourceFrame>> out;
   const auto t0 = tick();
   while(!source.is_finished() and tock(t0) < 10.0) {
      auto frames = source.take_all(100ms);
      out.insert(end(out), begin(frames), end(frames));
   }
   return out;
}

// Waits (in real time) until the clock thread has dealt with `n` frames
static bool wait_for_frames(const PacedFrameSource& source, uint64_t n)
{
   const auto t0 = tick();
   while(tock(t0) < 10.0) {
      const auto s = source.stats();
      if(s.produced + s.skipped_behind + s.decode_errors >= n) return true;
      std::this_thread::sleep_for(1ms);
   }
   return false;
}

static int pixel_frame_no(const SourceFrame& frame, int cam)
{
   return frame.images.raw_images.at(size_t(cam)).at<cv::Vec3b>(0, 0)[0];
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("PacedFrameSource", "[realtime_source]")
{
   CATCH_SECTION("unpaced-frames-in-order")
   {
      PacedFrameSource source(make_readers(3), make_params(20, 0.0, nullptr));
      source.start();
      const auto frames = drain(source);
      CATCH_REQUIRE(frames.size() == 20);
      for(int n = 0; n < 20; ++n) {
         const auto& f = *frames[size_t(n)];
         CATCH_REQUIRE(f.frame_no == n);
         CATCH_REQUIRE(!f.is_dropped());
         CATCH_REQUIRE(f.images.frame_loaded);
         CATCH_REQUIRE(f.images.raw_images.size() == 3);
         for(int cam = 0; cam < 3; ++cam) {
            CATCH_REQUIRE(pixel_frame_no(f, cam) == n);
            CATCH_REQUIRE(
                f.images.raw_images[size_t(cam)].at<cv::Vec3b>(0, 0)[1] == cam);
         }
         CATCH_REQUIRE(f.timestamp
                       == add_seconds(source.params().timestamp0, n / 15.0));
         CATCH_REQUIRE(f.t_arrival >= f.t_due);
      }
      CATCH_REQUIRE(source.stats().produced == 20);
      CATCH_REQUIRE(source.latest_frame_no() == 19);
      CATCH_REQUIRE(source.next(1ms) == nullptr);
   }

   CATCH_SECTION("paced-due-times")
   {
      FakeClock clock;
      const auto t0 = clock.now();
      PacedFrameSource source(make_readers(2), make_params(30, 10.0, &clock));
      source.start();
      const auto frames = drain(source);
      CATCH_REQUIRE(frames.size() == 30);
      for(int n = 0; n < 30; ++n) {
         const auto& f = *frames[size_t(n)];
         CATCH_REQUIRE(f.frame_no == n);
         CATCH_REQUIRE(!f.is_dropped());
         CATCH_REQUIRE(f.t_due == t0 + n * 100ms);
      }
      CATCH_REQUIRE(source.stats().skipped_behind == 0);
   }

   CATCH_SECTION("slow-camera-drops-whole-frames")
   {
      FakeClock clock;
      vector<unique_ptr<CameraReader>> readers;
      readers.push_back(make_unique<FakeCameraReader>(0, -1, &clock, 150ms));
      readers.push_back(make_unique<FakeCameraReader>(1));
      PacedFrameSource source(std::move(readers),
                              make_params(60, 15.0, &clock));
      source.start();
      const auto frames = drain(source);

      // Every frame number once, in order, decoded or dropped
      CATCH_REQUIRE(frames.size() == 60);
      int n_loaded = 0;
      for(int n = 0; n < 60; ++n) {
         const auto& f = *frames[size_t(n)];
         CATCH_REQUIRE(f.frame_no == n);
         if(f.is_dropped()) {
            CATCH_REQUIRE(f.drop_reason == DropReason::SOURCE_BEHIND);
            CATCH_REQUIRE(f.images.raw_images.empty());
         } else {
            ++n_loaded;
            CATCH_REQUIRE(pixel_frame_no(f, 0) == n);
            CATCH_REQUIRE(pixel_frame_no(f, 1) == n);
         }
      }
      const auto stats = source.stats();
      CATCH_REQUIRE(stats.skipped_behind > 0);
      CATCH_REQUIRE(stats.produced == uint64_t(n_loaded));
      CATCH_REQUIRE(stats.produced + stats.skipped_behind == 60);
      CATCH_REQUIRE(frames[0]->is_dropped() == false);
   }

   CATCH_SECTION("full-ring-drops-the-oldest")
   {
      FakeClock clock;
      auto p      = make_params(10, 15.0, &clock);
      p.ring_size = 2;
      PacedFrameSource source(make_readers(1), p);
      source.start();
      CATCH_REQUIRE(wait_for_frames(source, 10));
      const auto frames = drain(source);
      CATCH_REQUIRE(frames.size() == 10);
      for(int n = 0; n < 10; ++n) {
         CATCH_REQUIRE(frames[size_t(n)]->frame_no == n);
         CATCH_REQUIRE(frames[size_t(n)]->is_dropped() == (n < 8));
         if(n < 8)
            CATCH_REQUIRE(frames[size_t(n)]->drop_reason
                          == DropReason::NO_FREE_SLOT);
      }
      CATCH_REQUIRE(source.stats().evicted == 8);
      CATCH_REQUIRE(source.stats().produced == 10);
   }

   CATCH_SECTION("unpaced-waits-for-the-consumer")
   {
      auto p      = make_params(12, 0.0, nullptr);
      p.ring_size = 2;
      PacedFrameSource source(make_readers(2), p);
      source.start();
      std::this_thread::sleep_for(30ms);
      CATCH_REQUIRE(source.latest_frame_no() == 1); // the ring is full
      vector<int> seen;
      while(auto f = source.next(1000ms)) {
         CATCH_REQUIRE(!f->is_dropped());
         seen.push_back(f->frame_no);
         std::this_thread::sleep_for(2ms);
      }
      CATCH_REQUIRE(seen.size() == 12);
      for(size_t n = 0; n < seen.size(); ++n) CATCH_REQUIRE(seen[n] == int(n));
      CATCH_REQUIRE(source.stats().evicted == 0);
   }

   CATCH_SECTION("failed-camera-drops-the-frame")
   {
      PacedFrameSource source(make_readers(3, 1, 5),
                              make_params(10, 0.0, nullptr));
      source.start();
      const auto frames = drain(source);
      CATCH_REQUIRE(frames.size() == 10);
      for(int n = 0; n < 10; ++n) {
         CATCH_REQUIRE(frames[size_t(n)]->frame_no == n);
         CATCH_REQUIRE(frames[size_t(n)]->is_dropped() == (n == 5));
      }
      CATCH_REQUIRE(frames[5]->drop_reason == DropReason::DECODE_ERROR);
      CATCH_REQUIRE(source.stats().decode_errors == 1);
      CATCH_REQUIRE(source.stats().produced == 9);
   }

   CATCH_SECTION("tap-sees-every-kth-frame")
   {
      std::mutex padlock;
      vector<int> tapped;
      auto p       = make_params(13, 0.0, nullptr);
      p.tap_period = 3;
      p.tap        = [&](shared_ptr<const SourceFrame> f) {
         std::lock_guard<std::mutex> lock(padlock);
         tapped.push_back(f->frame_no);
      };
      PacedFrameSource source(make_readers(1), p);
      source.start();
      CATCH_REQUIRE(drain(source).size() == 13);
      std::lock_guard<std::mutex> lock(padlock);
      CATCH_REQUIRE(tapped == vector<int>{0, 3, 6, 9, 12});
   }

   CATCH_SECTION("split-function")
   {
      auto p  = make_params(4, 0.0, nullptr);
      p.split = [](vector<cv::Mat>&& raw, int frame_no) {
         SceneDescription::FrameImages ims;
         ims.frame_no     = frame_no;
         ims.frame_loaded = true;
         for(const auto& im : raw) {
            ims.sensor_images.push_back(im.colRange(0, im.cols / 2));
            ims.sensor_images.push_back(im.colRange(im.cols / 2, im.cols));
         }
         ims.raw_images = std::move(raw);
         return ims;
      };
      PacedFrameSource source(make_readers(2), p);
      source.start();
      const auto frames = drain(source);
      CATCH_REQUIRE(frames.size() == 4);
      for(const auto& f : frames) {
         CATCH_REQUIRE(f->images.frame_no == f->frame_no);
         CATCH_REQUIRE(f->images.sensor_images.size() == 4);
         CATCH_REQUIRE(f->images.sensor_images[0].cols == 3);
      }
   }

   CATCH_SECTION("stop-unblocks-waiters")
   {
      // Real clock, one frame every 2 seconds
      PacedFrameSource source(make_readers(2), make_params(10, 0.5, nullptr));
      source.start();
      CATCH_REQUIRE(source.next(1000ms) != nullptr); // frame 0 is due at once

      std::atomic<bool> returned = false;
      shared_ptr<const SourceFrame> got;
      std::thread waiter([&]() {
         got      = source.next(10000ms);
         returned = true;
      });
      std::this_thread::sleep_for(50ms);
      CATCH_REQUIRE(!returned);
      const auto t0 = tick();
      source.stop();
      waiter.join();
      CATCH_REQUIRE(tock(t0) < 0.1);
      CATCH_REQUIRE(got == nullptr);
      CATCH_REQUIRE(source.is_finished());
   }

   CATCH_SECTION("destructor-joins-promptly")
   {
      double seconds = 0.0;
      {
         auto source = make_unique<PacedFrameSource>(
             make_readers(4), make_params(100, 0.5, nullptr));
         source->start();
         std::this_thread::sleep_for(20ms);
         const auto t0 = tick();
         source.reset();
         seconds = tock(t0);
      }
      CATCH_REQUIRE(seconds < 0.1);
   }
}

// -------------------------------------------------------------- testcase video
// The first video of testcase 007, if it has been fetched
//
static string testcase_007_video() noexcept
{
   const char* data_dir = getenv("MULTIVIEW_TESTCASE_DATA");
   const string root    = (data_dir != nullptr)
                              ? string(data_dir)
                              : format("{}/multiview-data", getenv("HOME"));
   return format("{}/testcases/007_site03_sparse_tracks/C0001020_v1.mp4", root);
}

static string frame_digest(const cv::Mat& im) noexcept
{
   if(im.empty()) return ""s;
   Expects(im.isContinuous());
   return md5(im.data, im.total() * im.elemSize());
}

// ------------------------------------------------------------------- TEST_CASE
// The video reader gives the pictures that SceneDescription's seek gives:
// scene frame `k` is movie frame `k + offset` (identity mapping)
//
CATCH_TEST_CASE("PacedFrameSourceVideo", "[realtime_source]")
{
   const auto fname = testcase_007_video();
   if(!is_regular_file(fname)) {
      WARN(format("testcase video '{}' not found, skipping", fname));
      return;
   }

   constexpr int k_offset = 1; // the offset of every 007 camera
   int count              = 0;
   {
      cv::VideoCapture video(fname);
      count = int(video.get(cv::CAP_PROP_FRAME_COUNT));
   }
   CATCH_REQUIRE(count > 260);
   auto mapping = make_shared<vector<unsigned>>(size_t(count - k_offset));
   std::iota(begin(*mapping), end(*mapping), 0u);
   auto reader = make_video_camera_reader(fname, k_offset, count, mapping);

   auto seek_digest = [&](int frame_no) {
      cv::VideoCapture video(fname);
      cv::Mat im;
      video.set(cv::CAP_PROP_POS_FRAMES, frame_no + k_offset);
      video.retrieve(im);
      return frame_digest(im);
   };

   // Sequential, short gaps, backwards, a long gap, across the keyframe at 250
   const int last = count - k_offset - 1;
   for(const int frame_no : {0, 1, 2, 3, 7, 5, 30, 33, 249, 250, 251, last}) {
      CATCH_INFO(format("frame {}", frame_no));
      cv::Mat im;
      CATCH_REQUIRE(reader->read(frame_no, im));
      CATCH_REQUIRE(frame_digest(im) == seek_digest(frame_no));
   }

   cv::Mat im;
   CATCH_REQUIRE(!reader->read(last + 1, im));
   CATCH_REQUIRE(!reader->read(-1, im));
   CATCH_REQUIRE(reader->read(4, im)); // recovers after a failure
   CATCH_REQUIRE(frame_digest(im) == seek_digest(4));
}

} // namespace perceive::pipeline::realtime
