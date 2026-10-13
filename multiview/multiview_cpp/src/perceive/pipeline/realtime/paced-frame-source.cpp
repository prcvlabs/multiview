
#include "paced-frame-source.hpp"

#include "perceive/io/lazy-s3.hpp"
#include "perceive/utils/file-system.hpp"

#include <condition_variable>
#include <deque>
#include <thread>

#include <opencv2/videoio.hpp>

namespace perceive::pipeline::realtime
{
using std::chrono::steady_clock;

// -----------------------------------------------------------------------------
// --------------------------------------------------------------- Video Readers
// -----------------------------------------------------------------------------

// ------------------------------------------------------------ local-video-name
// `SceneDescription::init` has already fetched s3 videos to the lazy-s3 cache
//
static string local_video_name(const string& filename) noexcept
{
   if(begins_with(filename, string("s3://")))
      return lazy_s3_cache_filename(filename);
   return filename;
}

// ----------------------------------------------------------- VideoCameraReader
//
class VideoCameraReader final : public CameraReader
{
 private:
   // Read a gap of up to this many pictures by decoding through it
   static constexpr int k_max_grab_ahead = 8;

   string filename_;
   cv::VideoCapture video_;
   int movie_frame_offset_ = 0;
   int movie_frame_count_  = 0;
   shared_ptr<const vector<unsigned>> frame_mapping_;
   int next_picture_ = 0; // what `>>` returns next; -1 if unknown

 public:
   VideoCameraReader(const string& filename,
                     int movie_frame_offset,
                     int movie_frame_count,
                     shared_ptr<const vector<unsigned>> frame_mapping)
       : filename_(filename)
       , video_(local_video_name(filename))
       , movie_frame_offset_(movie_frame_offset)
       , movie_frame_count_(movie_frame_count)
       , frame_mapping_(std::move(frame_mapping))
   {
      if(!video_.isOpened())
         throw std::runtime_error(format("failed to open '{}'", filename));
      Expects(frame_mapping_ != nullptr);
   }

   // Seeking to movie frame `n` and retrieving gives the picture of movie
   // frame `n - 1` (`[video_decode]`), and `SceneDescription` seeks to
   // `frame_mapping[k] + offset`. Movie frame `fnum` is therefore the picture
   // `fnum - 1` of a plain decode from the start.
   bool read(int frame_no, cv::Mat& out) noexcept override
   {
      try {
         if(unsigned(frame_no) >= frame_mapping_->size()) return false;
         const int fnum
             = int(frame_mapping_->at(size_t(frame_no))) + movie_frame_offset_;
         if(fnum < 0 or fnum >= movie_frame_count_) return false;
         const int picture = std::max(fnum - 1, 0);

         const int gap = picture - next_picture_;
         bool ok       = false;
         if(next_picture_ >= 0 and gap >= 0 and gap <= k_max_grab_ahead) {
            ok = true;
            for(int i = 0; ok and i < gap; ++i) ok = video_.grab();
            ok = ok and video_.read(out);
         } else {
            video_.set(cv::CAP_PROP_POS_FRAMES, picture + 1);
            ok = video_.retrieve(out);
         }

         next_picture_ = (ok and !out.empty()) ? picture + 1 : -1;
         return next_picture_ >= 0;
      } catch(std::exception& e) {
         LOG_ERR(format("exception decoding frame {} of '{}': {}",
                        frame_no,
                        filename_,
                        e.what()));
      }
      next_picture_ = -1;
      return false;
   }
};

// ---------------------------------------------------- make-video-camera-reader
//
unique_ptr<CameraReader> make_video_camera_reader(
    const string& filename,
    int movie_frame_offset,
    int movie_frame_count,
    shared_ptr<const vector<unsigned>> frame_mapping) noexcept(false)
{
   return make_unique<VideoCameraReader>(filename,
                                         movie_frame_offset,
                                         movie_frame_count,
                                         std::move(frame_mapping));
}

// --------------------------------------------------- make-video-camera-readers
//
vector<unique_ptr<CameraReader>>
make_video_camera_readers(const VideoFrameMap& map) noexcept(false)
{
   const auto mapping = make_shared<const vector<unsigned>>(map.frame_mapping);
   vector<unique_ptr<CameraReader>> readers;
   for(auto i = 0; i < map.n_cameras(); ++i)
      readers.push_back(
          make_video_camera_reader(map.filenames.at(size_t(i)),
                                   map.movie_frame_offsets.at(size_t(i)),
                                   map.movie_frame_counts.at(size_t(i)),
                                   mapping));
   return readers;
}

// -------------------------------------------------------- make-video-frame-map
//
VideoFrameMap
make_video_frame_map(const SceneDescription& scene,
                     const SceneManifest& manifest) noexcept(false)
{
   const auto n_cameras = size_t(scene.n_cameras());
   if(manifest.timestamps.size() != manifest.size())
      throw std::runtime_error("expected one timestamp per manifest video");

   VideoFrameMap map;
   map.movie_fps = (manifest.fps.size() > 0) ? manifest.fps[0]
                                             : k_perceive_default_frame_rate;
   map.scene_fps = scene.scene_fps;
   map.filenames = scene.filenames;
   map.movie_frame_offsets.resize(n_cameras, 0);
   map.movie_frame_counts.resize(n_cameras, 0);

   for(size_t i = 0; i < n_cameras; ++i) {
      if(!scene.is_video.at(i))
         throw std::runtime_error(
             format("camera {} is not a video: '{}'", i, scene.filenames[i]));
      cv::VideoCapture video(local_video_name(scene.filenames[i]));
      if(!video.isOpened())
         throw std::runtime_error(
             format("failed to open '{}'", scene.filenames[i]));
      map.movie_frame_counts[i] = int(video.get(cv::CAP_PROP_FRAME_COUNT));
   }

   const auto lookup
       = calc_manifest_camera_lookup(manifest, scene.scene_info.bcam_keys);
   for(size_t i = 0; i < manifest.size(); ++i)
      map.movie_frame_offsets.at(size_t(lookup[i])) = calc_movie_frame_offset(
          map.movie_fps, scene.timestamp0, manifest.timestamps[i]);

   int n_movie_frames = std::numeric_limits<int>::max();
   for(size_t i = 0; i < n_cameras; ++i)
      n_movie_frames
          = std::min(n_movie_frames,
                     map.movie_frame_counts[i] - map.movie_frame_offsets[i]);
   if(n_cameras == 0) n_movie_frames = 1;
   if(n_movie_frames < 0)
      throw std::runtime_error("a video starts after another one ends");

   map.frame_mapping = scene_frame_to_movie_frame_mapping(
       map.movie_fps, map.scene_fps, n_movie_frames);

   if(map.n_scene_frames() != scene.n_frames())
      throw std::runtime_error(format("expected {} scene frames, but the "
                                      "videos give {}",
                                      scene.n_frames(),
                                      map.n_scene_frames()));
   return map;
}

// -----------------------------------------------------------------------------
// ------------------------------------------------------------ PacedFrameSource
// -----------------------------------------------------------------------------

struct PacedFrameSource::Pimpl
{
   Params params;
   vector<unique_ptr<CameraReader>> readers;

   // ------------------------------------ The ring, the stop flag and the stats
   mutable std::mutex padlock;
   std::condition_variable ring_changed; // wakes the consumer and the clock
   std::deque<shared_ptr<const SourceFrame>> ring;
   size_t n_loaded_in_ring = 0;
   bool is_stopped         = false;
   bool clock_done         = false;
   int latest_frame_no     = -1;
   Stats stats;

   // ------------------------------- One decode job at a time, for every camera
   std::mutex job_padlock;
   std::condition_variable job_posted;
   std::condition_variable job_done;
   uint64_t job_generation = 0;
   int job_frame_no        = -1;
   int n_jobs_done         = 0;
   bool jobs_stopped       = false;
   vector<cv::Mat> job_images;
   vector<char> job_ok;

   vector<std::thread> decoders;
   std::thread clock;
   bool is_started = false;

   Pimpl(vector<unique_ptr<CameraReader>>&& in_readers, const Params& p)
       : params(p)
       , readers(std::move(in_readers))
   {
      Expects(params.end_frame >= params.start_frame);
      Expects(params.ring_size > 0);
      Expects(params.tap_period > 0);
      Expects(params.scene_fps > 0.0);
      if(!params.now) params.now = []() { return steady_clock::now(); };
      job_images.resize(readers.size());
      job_ok.resize(readers.size(), 0);
   }

   bool is_paced() const noexcept { return params.rate_fps > 0.0; }
   bool may_drop() const noexcept
   {
      return is_paced() and params.drop_when_behind;
   }

   bool stopped() const noexcept
   {
      std::lock_guard<std::mutex> lock(padlock);
      return is_stopped;
   }

   // -------------------------------------------------------------- sleep-until
   //
   void sleep_until(time_point t) noexcept
   {
      if(params.sleep_until) {
         params.sleep_until(t);
         return;
      }
      std::unique_lock<std::mutex> lock(padlock);
      ring_changed.wait_until(lock, t, [this]() { return is_stopped; });
   }

   // ---------------------------------------------------------------- push-drop
   //
   void push_drop(int frame_no, time_point t_due, DropReason reason) noexcept
   {
      auto frame         = make_shared<SourceFrame>();
      frame->frame_no    = frame_no;
      frame->timestamp   = timestamp_of(frame_no);
      frame->t_due       = t_due;
      frame->t_arrival   = params.now();
      frame->drop_reason = reason;
      {
         std::lock_guard<std::mutex> lock(padlock);
         if(reason == DropReason::SOURCE_BEHIND) ++stats.skipped_behind;
         if(reason == DropReason::DECODE_ERROR) ++stats.decode_errors;
         ring.push_back(std::move(frame));
      }
      ring_changed.notify_all();
   }

   // --------------------------------------------------------------- push-frame
   // Blocks while the ring is full, unless frames may be dropped, in which
   // case the oldest decoded frame in the ring becomes a drop record. Returns
   // false if stopped first.
   bool push_frame(shared_ptr<const SourceFrame> frame) noexcept
   {
      {
         std::unique_lock<std::mutex> lock(padlock);
         if(!may_drop())
            ring_changed.wait(lock, [this]() {
               return is_stopped or n_loaded_in_ring < params.ring_size;
            });
         if(is_stopped) return false;

         while(n_loaded_in_ring >= params.ring_size) {
            auto ii = std::find_if(begin(ring), end(ring), [](const auto& o) {
               return !o->is_dropped();
            });
            Expects(ii != end(ring));
            auto evicted         = make_shared<SourceFrame>();
            evicted->frame_no    = (*ii)->frame_no;
            evicted->timestamp   = (*ii)->timestamp;
            evicted->t_due       = (*ii)->t_due;
            evicted->t_arrival   = (*ii)->t_arrival;
            evicted->drop_reason = DropReason::NO_FREE_SLOT;
            *ii                  = std::move(evicted);
            --n_loaded_in_ring;
            ++stats.evicted;
         }

         latest_frame_no = frame->frame_no;
         ring.push_back(std::move(frame));
         ++n_loaded_in_ring;
         ++stats.produced;
      }
      ring_changed.notify_all();
      return true;
   }

   // ------------------------------------------------------------- timestamp-of
   //
   Timestamp timestamp_of(int frame_no) const noexcept
   {
      return add_seconds(params.timestamp0, frame_no / params.scene_fps);
   }

   // ------------------------------------------------------------------- decode
   // Every camera decodes `frame_no` on its own thread. False if stopped, or
   // if any camera failed.
   bool decode(int frame_no, vector<cv::Mat>& out) noexcept
   {
      std::unique_lock<std::mutex> lock(job_padlock);
      job_frame_no = frame_no;
      n_jobs_done  = 0;
      ++job_generation;
      job_posted.notify_all();
      job_done.wait(lock, [this]() {
         return jobs_stopped or n_jobs_done == int(readers.size());
      });
      if(jobs_stopped) return false;
      out = std::move(job_images);
      job_images.resize(readers.size());
      return std::all_of(
          cbegin(job_ok), cend(job_ok), [](char ok) { return ok != 0; });
   }

   // -------------------------------------------------------------- run-decoder
   //
   void run_decoder(size_t cam) noexcept
   {
      uint64_t generation = 0;
      while(true) {
         int frame_no = -1;
         {
            std::unique_lock<std::mutex> lock(job_padlock);
            job_posted.wait(lock, [&]() {
               return jobs_stopped or job_generation != generation;
            });
            if(jobs_stopped) return;
            generation = job_generation;
            frame_no   = job_frame_no;
         }

         cv::Mat im;
         const bool ok = readers[cam]->read(frame_no, im);

         {
            std::lock_guard<std::mutex> lock(job_padlock);
            job_images[cam] = std::move(im);
            job_ok[cam]     = (ok and !job_images[cam].empty()) ? 1 : 0;
            ++n_jobs_done;
         }
         job_done.notify_all();
      }
   }

   // ---------------------------------------------------------------- run-clock
   //
   void run_clock() noexcept
   {
      const auto t0 = params.now();
      auto seconds  = [](double s) {
         return std::chrono::round<steady_clock::duration>(
             std::chrono::duration<double>(s));
      };
      const auto period = seconds(is_paced() ? 1.0 / params.rate_fps : 0.0);

      for(int k = params.start_frame; k < params.end_frame; ++k) {
         if(stopped()) break;

         time_point t_due = params.now();
         if(is_paced()) {
            t_due = t0 + seconds((k - params.start_frame) / params.rate_fps);
            sleep_until(t_due);
            if(stopped()) break;
            if(may_drop() and params.now() >= t_due + period) {
               push_drop(k, t_due, DropReason::SOURCE_BEHIND);
               continue;
            }
         }

         vector<cv::Mat> raw;
         const bool ok = decode(k, raw);
         if(stopped()) break;
         if(!ok) {
            push_drop(k, t_due, DropReason::DECODE_ERROR);
            continue;
         }

         auto frame       = make_shared<SourceFrame>();
         frame->frame_no  = k;
         frame->timestamp = timestamp_of(k);
         frame->t_due     = t_due;
         if(params.split) {
            try {
               frame->images = params.split(std::move(raw), k);
            } catch(std::exception& e) {
               LOG_ERR(format("exception splitting frame {}: {}", k, e.what()));
               push_drop(k, t_due, DropReason::DECODE_ERROR);
               continue;
            }
         } else {
            frame->images.frame_no     = k;
            frame->images.raw_images   = std::move(raw);
            frame->images.frame_loaded = true;
         }
         frame->t_arrival = params.now();

         shared_ptr<const SourceFrame> cframe = frame;
         if(!push_frame(cframe)) break;
         if(params.tap and k % int(params.tap_period) == 0) {
            try {
               params.tap(cframe);
            } catch(std::exception& e) {
               LOG_ERR(format("exception in the frame tap: {}", e.what()));
            }
         }
      }

      {
         std::lock_guard<std::mutex> lock(padlock);
         clock_done = true;
      }
      ring_changed.notify_all();
   }

   // --------------------------------------------------------- pop-front-locked
   // The caller holds `padlock`
   shared_ptr<const SourceFrame> pop_front_locked() noexcept
   {
      auto frame = std::move(ring.front());
      ring.pop_front();
      if(!frame->is_dropped()) --n_loaded_in_ring;
      return frame;
   }

   // ------------------------------------------------------------- wait-for-any
   // The caller holds `lock`. True if there is something in the ring.
   bool wait_for_any(std::unique_lock<std::mutex>& lock,
                     std::chrono::milliseconds timeout) noexcept
   {
      ring_changed.wait_for(lock, timeout, [this]() {
         return is_stopped or clock_done or !ring.empty();
      });
      return !is_stopped and !ring.empty();
   }
};

// ---------------------------------------------------------------- Construction
//
PacedFrameSource::PacedFrameSource(vector<unique_ptr<CameraReader>> readers,
                                   const Params& params)
    : pimpl_(make_unique<Pimpl>(std::move(readers), params))
{}

PacedFrameSource::~PacedFrameSource() { stop(); }

// ---------------------------------------------------------------------- params
//
const PacedFrameSource::Params& PacedFrameSource::params() const noexcept
{
   return pimpl_->params;
}

// ------------------------------------------------------------------- n-cameras
//
int PacedFrameSource::n_cameras() const noexcept
{
   return int(pimpl_->readers.size());
}

// ----------------------------------------------------------------------- stats
//
PacedFrameSource::Stats PacedFrameSource::stats() const noexcept
{
   std::lock_guard<std::mutex> lock(pimpl_->padlock);
   return pimpl_->stats;
}

// ------------------------------------------------------------- latest-frame-no
//
int PacedFrameSource::latest_frame_no() const noexcept
{
   std::lock_guard<std::mutex> lock(pimpl_->padlock);
   return pimpl_->latest_frame_no;
}

// ----------------------------------------------------------------- is-finished
//
bool PacedFrameSource::is_finished() const noexcept
{
   std::lock_guard<std::mutex> lock(pimpl_->padlock);
   return pimpl_->is_stopped or (pimpl_->clock_done and pimpl_->ring.empty());
}

// ----------------------------------------------------------------------- start
//
void PacedFrameSource::start() noexcept
{
   auto& P = *pimpl_;
   if(P.is_started) return;
   P.is_started = true;
   for(size_t i = 0; i < P.readers.size(); ++i)
      P.decoders.emplace_back([&P, i]() { P.run_decoder(i); });
   P.clock = std::thread([&P]() { P.run_clock(); });
}

// ------------------------------------------------------------------------ stop
//
void PacedFrameSource::stop() noexcept
{
   auto& P = *pimpl_;
   {
      std::lock_guard<std::mutex> lock(P.padlock);
      P.is_stopped = true;
   }
   P.ring_changed.notify_all();
   {
      std::lock_guard<std::mutex> lock(P.job_padlock);
      P.jobs_stopped = true;
   }
   P.job_posted.notify_all();
   P.job_done.notify_all();

   if(P.clock.joinable()) P.clock.join();
   for(auto& t : P.decoders)
      if(t.joinable()) t.join();
}

// ------------------------------------------------------------------------ next
//
shared_ptr<const SourceFrame>
PacedFrameSource::next(std::chrono::milliseconds timeout)
{
   auto& P = *pimpl_;
   shared_ptr<const SourceFrame> frame;
   {
      std::unique_lock<std::mutex> lock(P.padlock);
      if(!P.wait_for_any(lock, timeout)) return nullptr;
      frame = P.pop_front_locked();
   }
   P.ring_changed.notify_all();
   return frame;
}

// -------------------------------------------------------------------- take-all
//
vector<shared_ptr<const SourceFrame>>
PacedFrameSource::take_all(std::chrono::milliseconds timeout)
{
   auto& P = *pimpl_;
   vector<shared_ptr<const SourceFrame>> frames;
   {
      std::unique_lock<std::mutex> lock(P.padlock);
      if(!P.wait_for_any(lock, timeout)) return frames;
      while(!P.ring.empty()) frames.push_back(P.pop_front_locked());
   }
   P.ring_changed.notify_all();
   return frames;
}

} // namespace perceive::pipeline::realtime
