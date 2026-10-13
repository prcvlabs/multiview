
#pragma once

#include "frame-record.hpp"

#include "perceive/scene/scene-description.hpp"

#include <chrono>

namespace perceive::pipeline::realtime
{
// ----------------------------------------------------------------- SourceFrame
// One scene frame, decoded on every camera at once, or the record of a scene
// frame that was dropped before it was decoded or delivered.
//
struct SourceFrame
{
   using time_point = std::chrono::steady_clock::time_point;

   int frame_no           = -1;
   Timestamp timestamp    = {}; // capture time
   time_point t_due       = {}; // when the paced clock released the frame
   time_point t_arrival   = {}; // when every camera had decoded it
   DropReason drop_reason = DropReason::NONE;
   SceneDescription::FrameImages images = {}; // empty when dropped

   bool is_dropped() const noexcept { return drop_reason != DropReason::NONE; }
};

// ---------------------------------------------------------------- CameraReader
// Decodes the pictures of one camera. A reader is used by one thread at a
// time.
//
class CameraReader
{
 public:
   virtual ~CameraReader() = default;

   // The picture of scene frame `frame_no`; false if it cannot be read
   virtual bool read(int frame_no, cv::Mat& out) noexcept = 0;
};

// --------------------------------------------------------------- VideoFrameMap
// How the scene frames of a manifest map onto the frames of its videos,
// computed as `SceneDescription::init` does: scene frame `k` of camera `c` is
// movie frame `frame_mapping[k] + movie_frame_offsets[c]`.
//
struct VideoFrameMap
{
   real movie_fps = 0.0;
   real scene_fps = 0.0;
   vector<string> filenames;        // one per scene camera
   vector<int> movie_frame_offsets; // one per scene camera
   vector<int> movie_frame_counts;  // one per scene camera
   vector<unsigned> frame_mapping;  // scene frame => movie frame

   int n_cameras() const noexcept { return int(filenames.size()); }
   int n_scene_frames() const noexcept { return int(frame_mapping.size()); }
};

// Opens every video of `scene` to count its frames. Throws
// std::runtime_error when a camera is not a video, or a video cannot be read,
// or the result disagrees with `scene.n_frames()`.
VideoFrameMap
make_video_frame_map(const SceneDescription& scene,
                     const SceneManifest& manifest) noexcept(false);

// Reads `filename` as `SceneDescription` does, with its own cv::VideoCapture:
// consecutive frames with `>>`, a short gap by grabbing the frames between,
// and anything else by seeking. Throws if the video cannot be opened.
unique_ptr<CameraReader> make_video_camera_reader(
    const string& filename,
    int movie_frame_offset,
    int movie_frame_count,
    shared_ptr<const vector<unsigned>> frame_mapping) noexcept(false);

vector<unique_ptr<CameraReader>>
make_video_camera_readers(const VideoFrameMap& map) noexcept(false);

// ------------------------------------------------------------ PacedFrameSource
// Replays scene frames at a wall-clock rate. A clock thread releases frame
// `k` at `t0 + (k - start_frame) / rate_fps`, one decoder thread per camera
// decodes it, and the clock thread splits the pictures into sensor images and
// appends the frame to a ring. A frame that is a whole period late is dropped
// (`SOURCE_BEHIND`) before it is decoded, a frame that a camera cannot decode
// is dropped (`DECODE_ERROR`), and when the ring is full the oldest frame in
// it is dropped (`NO_FREE_SLOT`). Drops are delivered in frame order, as
// frames without images, so the consumer sees every frame number once.
//
// With `rate_fps == 0`, or without `drop_when_behind`, nothing is dropped
// for being late: the clock waits for the consumer instead.
//
// No thread of this class runs jobs of the global thread pool.
//
class PacedFrameSource
{
 public:
   using time_point = std::chrono::steady_clock::time_point;
   using split_function
       = std::function<SceneDescription::FrameImages(vector<cv::Mat>&& raw,
                                                     int frame_no)>;
   using tap_function = std::function<void(shared_ptr<const SourceFrame>)>;

   struct Params
   {
      int start_frame       = 0;
      int end_frame         = 0;    // exclusive
      real scene_fps        = 15.0; // for the frames' timestamps
      Timestamp timestamp0  = {};   // of scene frame 0
      real rate_fps         = 0.0;  // 0: as fast as the consumer takes them
      unsigned ring_size    = 6;    // decoded frames held for the consumer
      bool drop_when_behind = true;

      // Defaults: std::chrono::steady_clock, and a wait that `stop()` ends
      std::function<time_point()> now;
      std::function<void(time_point)> sleep_until;

      // Default: `images.raw_images` only, without sensor images
      split_function split;

      // Called on the clock thread for every frame that enters the ring and
      // whose number is a multiple of `tap_period`. Must not block for long.
      tap_function tap;
      unsigned tap_period = 1;
   };

   struct Stats
   {
      uint64_t produced       = 0; // frames that entered the ring
      uint64_t skipped_behind = 0; // SOURCE_BEHIND
      uint64_t decode_errors  = 0; // DECODE_ERROR
      uint64_t evicted        = 0; // NO_FREE_SLOT
   };

 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   PacedFrameSource(vector<unique_ptr<CameraReader>> readers,
                    const Params& params);
   PacedFrameSource(const PacedFrameSource&) = delete;
   PacedFrameSource(PacedFrameSource&&)      = delete;
   ~PacedFrameSource(); // stop(), and joins every thread
   PacedFrameSource& operator=(const PacedFrameSource&) = delete;
   PacedFrameSource& operator=(PacedFrameSource&&)      = delete;

   const Params& params() const noexcept;
   int n_cameras() const noexcept;
   Stats stats() const noexcept;
   int latest_frame_no() const noexcept; // last decoded frame in, or -1
   bool is_finished() const noexcept;    // the clock is done, the ring empty

   void start() noexcept; // spawns the decoder threads and the clock thread
   void stop() noexcept;  // idempotent; wakes every waiter

   // The oldest frame or drop record, waiting up to `timeout`. nullptr on a
   // timeout, after `stop()`, or at the end of the stream.
   shared_ptr<const SourceFrame> next(std::chrono::milliseconds timeout);

   // Everything that is waiting, oldest first, waiting up to `timeout` for
   // there to be something.
   vector<shared_ptr<const SourceFrame>>
   take_all(std::chrono::milliseconds timeout);
};

} // namespace perceive::pipeline::realtime
