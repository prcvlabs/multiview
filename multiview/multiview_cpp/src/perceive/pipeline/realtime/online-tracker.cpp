
#include "online-tracker.hpp"

#include "bounded-queue.hpp"
#include "latency-stats.hpp"
#include "track-stream.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/tracks/ops.hpp"

#include <deque>
#include <map>
#include <thread>

namespace perceive::pipeline::realtime
{
using std::chrono::steady_clock;
using time_point = steady_clock::time_point;
using tracks::NodeSequence;

// ------------------------------------------------------------------ ms-between
//
static double ms_between(time_point a, time_point b) noexcept
{
   return std::chrono::duration<double, std::milli>(b - a).count();
}

// ------------------------------------------------------------ addresses-of-seq
// The detections of a sequence's real (not interpolated) nodes, sorted
//
static vector<P2dAddress> addresses_of_seq(const NodeSequence& seq)
{
   vector<P2dAddress> out;
   for(const auto& node : seq.nodes())
      if(!node.is_interp())
         out.insert(end(out), cbegin(node.p2ds()), cend(node.p2ds()));
   std::sort(begin(out), end(out));
   return out;
}

// --------------------------------------------------------------- calc-id-remap
// `{old, new}` for every sequence of `before` whose detections are, for the
// most part, in a sequence of `after` with another id
//
static vector<std::pair<int, int>>
calc_id_remap(const vector<NodeSequence>& before,
              const vector<NodeSequence>& after)
{
   vector<vector<P2dAddress>> after_addresses;
   after_addresses.reserve(after.size());
   for(const auto& seq : after)
      after_addresses.push_back(addresses_of_seq(seq));

   vector<std::pair<int, int>> out;
   for(const auto& seq : before) {
      const auto addresses = addresses_of_seq(seq);
      if(addresses.empty()) continue;
      int best_id      = -1;
      size_t best_size = 0;
      for(size_t i = 0; i < after.size(); ++i) {
         vector<P2dAddress> common;
         std::set_intersection(cbegin(addresses),
                               cend(addresses),
                               cbegin(after_addresses[i]),
                               cend(after_addresses[i]),
                               std::back_inserter(common));
         const bool is_better = (common.size() > best_size)
                                or (common.size() == best_size and best_size > 0
                                    and after[i].id() < best_id);
         if(is_better) {
            best_size = common.size();
            best_id   = after[i].id();
         }
      }
      if(best_size > 0 and best_id != seq.id())
         out.emplace_back(seq.id(), best_id);
   }
   std::sort(begin(out), end(out));
   return out;
}

// ----------------------------------------------------------------------- Pimpl
//
struct OnlineTracker::Pimpl
{
   // ------------------------------------------------------------ Construction
   shared_ptr<const SceneDescription> scene;
   Params p;
   publish_fn publish;
   window_fn window;
   observer_fn observer;
   RealtimeStats* stats = nullptr;
   int K                = 0;
   int n_sensors        = 0;
   int n_cameras        = 0;
   int scene_n_frames   = 0;
   bool is_batch_window = false;

   BoundedQueue<FrameRecord> queue;
   std::thread thread;
   std::atomic<bool> has_error = false;
   std::atomic<int> n_windows  = 0;
   bool is_finished            = false;

   // ----------------------------------------------------- Tracker thread only
   bool is_failed = false;
   int next_frame = 0;
   bool has_good  = false;
   FrameRecord last_good;          // the last frame with a result
   std::deque<FrameRecord> frames; // the window being gathered
   std::deque<Tracks> windows;     // `push_back` keeps `prev` stable
   std::deque<shared_ptr<const Tracklet>> tracklets; // the last two
   std::deque<vector<FrameTimeline>> timelines;      // the last two windows
   std::deque<FrameTimeline> not_final; // revised, smoothing not final
   vector<NodeSequence> provisional;    // `windows.back().seqs` when published
   TrackStream stream;
   TrackerResult header; // what `calc_fp_info` reads besides the sequence

   // ----------------------------------------------------------------- Results
   int n_frames_pushed = 0;
   TrackerResult raw;
   FowlkesResult presmooth;
   FowlkesResult final_result;

   Pimpl(shared_ptr<const SceneDescription> in_scene,
         const Params& in_p,
         publish_fn in_publish,
         window_fn in_window,
         observer_fn in_observer,
         RealtimeStats* in_stats)
       : scene(std::move(in_scene))
       , p(in_p)
       , publish(std::move(in_publish))
       , window(std::move(in_window))
       , observer(std::move(in_observer))
       , stats(in_stats)
       , K(int(in_p.tracklet_params.max_frames_per_tracklet))
       , queue(size_t(std::max(1, K)))
       , next_frame(in_p.start_frame)
       , stream(int(in_p.fowlkes_params.smooth_kz_filter_m),
                int(in_p.fowlkes_params.smooth_kz_filter_k))
   {
      Expects(scene != nullptr);
      Expects(K > 0);
      Expects(p.start_frame >= 0 and p.start_frame < p.end_frame);
      n_sensors      = scene->n_sensors();
      n_cameras      = scene->n_cameras();
      scene_n_frames = scene->n_frames();

      is_batch_window = !window;
      if(is_batch_window) {
         Expects(p.end_frame <= scene_n_frames);
         window = [this](const Tracklet* tracklet,
                         const Tracklet* prev_tracklet,
                         Tracks* prev_track) {
            return tracks::calc_tracks_comp_data(
                       *scene,
                       p.loc_params,
                       p.tracklet_params,
                       p.tracks_params,
                       p.frame_duration,
                       K,
                       tracklet,
                       prev_tracklet,
                       prev_track,
                       []() { return false; },
                       p.outdir,
                       false)
                ->execute();
         };
      }

      header.frame_duration = p.frame_duration;
   }

   int window_last(int frame_no) const noexcept
   {
      return std::min((frame_no / K + 1) * K, p.end_frame) - 1;
   }

   // ------------------------------------------------------------------ fail
   //
   void fail(int window_idx, const string& reason) noexcept
   {
      LOG_ERR(format("tracking window {}: {}", window_idx, reason));
      is_failed = true;
      has_error = true;
      WindowErrorMessage msg;
      msg.window = window_idx;
      msg.reason = reason;
      publish(std::move(msg));
   }

   // ---------------------------------------------------------------- fp-info
   // The batch keep/drop rule on the sequence so far; not a true positive if
   // a node has no `prob_fp` (where `calc_fp_info` would not return)
   TrackFalsePositiveInfo fp_info(const NodeSequence& seq) const
   {
      bool is_valid = seq.size() > 0;
      for(const auto& node : seq.nodes())
         if(!node.is_interp() and !std::isfinite(node.prob_fp()))
            is_valid = false;
      return is_valid ? calc_fp_info(header, p.post_process, &seq)
                      : TrackFalsePositiveInfo{};
   }

   // ----------------------------------------------------------------- to-X
   //
   Vector2 to_X(const Vector2f& xy) const noexcept
   {
      return FloorHistogram::unproject_hist_xy(
          xy, header.top_left, header.hist_sz);
   }

   // --------------------------------------------------------- frame-points
   // The nodes of `seqs` at frames [start, start + n), by frame
   vector<vector<TrackPoint>>
   frame_points(const vector<NodeSequence>& seqs,
                int start,
                int n,
                const std::map<int, TrackFalsePositiveInfo>& infos) const
   {
      vector<vector<TrackPoint>> out(size_t(std::max(0, n)));
      for(const auto& seq : seqs) {
         const auto ii = infos.find(seq.id());
         for(const auto& node : seq.nodes()) {
            const int i = node.t() - start;
            if(i < 0 or i >= n) continue;
            const auto X = to_X(node.xy());
            TrackPoint o;
            o.id     = seq.id();
            o.x      = X.x;
            o.y      = X.y;
            o.gaze   = real(node.gaze());
            o.interp = node.is_interp();
            if(ii != cend(infos)) {
               o.confirmed = ii->second.is_true_positive;
               o.score     = real(ii->second.score);
            }
            out[size_t(i)].push_back(o);
         }
      }
      for(auto& points : out)
         std::sort(begin(points),
                   end(points),
                   [](const auto& a, const auto& b) { return a.id < b.id; });
      return out;
   }

   // ------------------------------------------------------- publish-tracked
   // Revision 0 of the newest window
   void publish_tracked(const Tracks& tracks,
                        const vector<FrameTimeline>& frame_timelines)
   {
      std::map<int, TrackFalsePositiveInfo> infos;
      for(const auto& seq : tracks.seqs) {
         const auto* prefix = stream.sequence(seq.id());
         if(prefix != nullptr and prefix->size() > 0 and seq.size() > 0
            and prefix->nodes().back().t() + 1 == seq.nodes().front().t()) {
            auto whole = *prefix;
            whole.append(seq);
            infos[seq.id()] = fp_info(whole);
         } else {
            infos[seq.id()] = fp_info(seq);
         }
      }

      const int idx   = tracks.start_frame / K;
      const auto ppts = frame_points(
          tracks.seqs, tracks.start_frame, tracks.n_frames, infos);
      const auto now = steady_clock::now();
      for(int i = 0; i < tracks.n_frames; ++i) {
         TracksMessage msg;
         msg.window   = idx;
         msg.frame    = tracks.start_frame + i;
         msg.revision = 0;
         msg.status   = "tracked"s;
         msg.tracks   = ppts[size_t(i)];
         publish(std::move(msg));
      }

      if(stats == nullptr) return;
      for(const auto& timeline : frame_timelines) {
         stats->count_frame(RealtimeStats::Stage::TRACKED, now);
         if(timeline.t_arrival != time_point{})
            stats->add_latency(RealtimeStats::Latency::TRACKED,
                               ms_between(timeline.t_arrival, now));
      }
   }

   // ------------------------------------------------------- publish-revised
   // Revision 1 of `revised`, which `newest` has just rewritten (`newest`
   // is null at the end of the stream), with the smoothed points and the
   // tracks that ended
   void publish_revised(const Tracks& revised,
                        const Tracks* newest,
                        const vector<std::pair<int, int>>& id_remap,
                        const vector<FrameTimeline>& frame_timelines)
   {
      const int start    = revised.start_frame;
      const int n        = revised.n_frames;
      const int last_t   = start + n - 1;
      const auto updates = stream.update(revised, newest, last_t);

      std::map<int, TrackFalsePositiveInfo> infos;
      for(const auto& seq : revised.seqs) {
         const auto* whole = stream.sequence(seq.id());
         infos[seq.id()]   = fp_info((whole == nullptr) ? seq : *whole);
      }

      const auto ppts = frame_points(revised.seqs, start, n, infos);
      for(int i = 0; i < n; ++i) {
         TracksMessage msg;
         msg.window   = start / K;
         msg.frame    = start + i;
         msg.revision = 1;
         msg.status   = "revised"s;
         msg.id_remap = id_remap;
         msg.tracks   = ppts[size_t(i)];
         publish(std::move(msg));
      }

      for(const auto& update : updates) {
         SmoothedMessage msg;
         msg.id           = update.id;
         msg.final_upto_t = update.final_upto_t;
         msg.points.reserve(update.track.path.size());
         for(const auto& tp : update.track.path) {
            const auto X = to_X(Vector2f(tp.x, tp.y));
            SmoothedPoint o;
            o.x    = X.x;
            o.y    = X.y;
            o.t    = tp.t;
            o.gaze = real(tp.gaze_direction);
            msg.points.push_back(o);
         }
         publish(std::move(msg));
      }

      for(const auto& update : updates) {
         if(!update.is_ended) continue;
         const auto* seq = stream.sequence(update.id);
         if(seq == nullptr or seq->size() == 0) continue;
         TrackEndMessage msg;
         msg.id        = update.id;
         msg.last_t    = seq->nodes().back().t();
         msg.confirmed = fp_info(*seq).is_true_positive;
         publish(std::move(msg));
      }

      // Latencies: revised now, and final up to the smoothing horizon
      const auto now = steady_clock::now();
      for(const auto& timeline : frame_timelines) {
         if(stats != nullptr and timeline.t_arrival != time_point{})
            stats->add_latency(RealtimeStats::Latency::REVISED,
                               ms_between(timeline.t_arrival, now));
         not_final.push_back(timeline);
      }
      const int horizon = (newest == nullptr) ? std::numeric_limits<int>::max()
                                              : last_t - stream.delta_t();
      while(!not_final.empty() and not_final.front().frame_no <= horizon) {
         const auto& timeline = not_final.front();
         if(stats != nullptr and timeline.t_arrival != time_point{})
            stats->add_latency(RealtimeStats::Latency::FINAL,
                               ms_between(timeline.t_arrival, now));
         not_final.pop_front();
      }
   }

   // --------------------------------------------------------------- validate
   // Empty if the window in `frames` may be given to the window code;
   // otherwise why not. These are the window code's preconditions.
   string validate(int t0, int n) const
   {
      if(n <= 0 or n > K)
         return format("{} frames in a window of at most {}", n, K);
      if((t0 + n - 1) / K != t0 / K)
         return format("frames [{}, {}] are in two windows", t0, t0 + n - 1);
      if(t0 + n > p.end_frame)
         return format("frame {} is past the end, {}", t0 + n - 1, p.end_frame);
      if(is_batch_window and t0 + n > scene_n_frames)
         return format("frame {} is past the scene's {} frames",
                       t0 + n - 1,
                       scene_n_frames);
      if(!windows.empty()) {
         const auto& prev = windows.back();
         if(t0 != prev.start_frame + prev.n_frames)
            return format("start_frame {} != prev {} + {}",
                          t0,
                          prev.start_frame,
                          prev.n_frames);
      }

      const auto& first = frames.front().loc_ptr;
      for(int i = 0; i < n; ++i) {
         const auto& rec = frames[size_t(i)];
         if(rec.frame_no() != t0 + i)
            return format(
                "frame {} where frame {} was expected", rec.frame_no(), t0 + i);
         if(rec.loc_ptr == nullptr or first == nullptr)
            return format("frame {} has no localization", rec.frame_no());
         if(!rec.loc_ptr->invariant_check())
            return format("frame {}'s localization is invalid", rec.frame_no());
         if(rec.loc_ptr->hist_sz != first->hist_sz
            or rec.loc_ptr->loc_hist.width != first->loc_hist.width
            or rec.loc_ptr->loc_hist.height != first->loc_hist.height)
            return format("frame {}'s histogram differs from frame {}'s",
                          rec.frame_no(),
                          t0);
         if(int(rec.labs.size()) != n_sensors)
            return format("frame {} has {} LAB images for {} sensors",
                          rec.frame_no(),
                          rec.labs.size(),
                          n_sensors);
         for(const auto& lab : rec.labs)
            if(lab == nullptr)
               return format("frame {} has a null LAB image", rec.frame_no());
         if(int(rec.point_clouds.size()) != n_cameras)
            return format("frame {} has {} point clouds for {} cameras",
                          rec.frame_no(),
                          rec.point_clouds.size(),
                          n_cameras);
         for(const auto& cloud : rec.point_clouds)
            if(cloud == nullptr)
               return format("frame {} has a null point cloud", rec.frame_no());
      }
      return ""s;
   }

   // ----------------------------------------------------------- fill-gaps
   // Gives every dropped frame of `frames` an empty localization and the
   // images of the last good frame (or, before any, of the window's first
   // good frame). False if there is no good frame to borrow from.
   bool fill_gaps()
   {
      const FrameRecord* like = has_good ? &last_good : nullptr;
      if(like == nullptr)
         for(const auto& rec : frames)
            if(!rec.is_dropped() and rec.loc_ptr != nullptr) {
               like = &rec;
               break;
            }
      if(like == nullptr) return false;

      for(auto& rec : frames) {
         if(!rec.is_dropped() and rec.loc_ptr != nullptr) {
            if(!has_good) {
               header.top_left = rec.loc_ptr->bounds.top_left();
               header.hist_sz  = rec.loc_ptr->hist_sz;
            }
            last_good = rec;
            has_good  = true;
            like      = &last_good;
            continue;
         }
         rec.loc_ptr
             = make_shared<const LocalizationData>(LocalizationData::make_empty(
                 *like->loc_ptr, p.loc_params, rec.frame_no()));
         rec.labs         = like->labs;
         rec.point_clouds = like->point_clouds;
      }
      return true;
   }

   // ----------------------------------------------------------- close-window
   //
   void close_window()
   {
      const int t0  = frames.front().frame_no();
      const int n   = int(frames.size());
      const int idx = t0 / K;

      if(!fill_gaps()) { // Before the first good frame: not tracked
         INFO(format("tracking window {} has no frame with a result; it is "
                     "not tracked",
                     idx));
         frames.clear();
         return;
      }

      const string error = validate(t0, n);
      if(!error.empty()) {
         fail(idx, error);
         frames.clear();
         return;
      }

      const auto t_start  = steady_clock::now();
      const bool has_prev = !windows.empty();
      shared_ptr<const Tracklet> tracklet;
      Tracks out;
      try {
         auto get_loc
             = [this, t0](int t) { return frames[size_t(t - t0)].loc_ptr; };
         auto get_lab = [this, t0](int t, int sensor_no) {
            return frames[size_t(t - t0)].labs[size_t(sensor_no)];
         };
         auto get_pt_cloud = [this, t0](int t, int camera_no) {
            return frames[size_t(t - t0)].point_clouds[size_t(camera_no)];
         };
         tracklet = make_shared<const Tracklet>(
             Tracklet::execute(*scene,
                               p.tracklet_params,
                               t0,
                               n,
                               K,
                               false,
                               get_loc,
                               get_lab,
                               get_pt_cloud,
                               []() { return false; }));

         if(has_prev) provisional = windows.back().seqs;
         out = window(tracklet.get(),
                      has_prev ? tracklets.back().get() : nullptr,
                      has_prev ? &windows.back() : nullptr);
      } catch(std::exception& e) {
         fail(idx, format("exception: {}", e.what()));
         frames.clear();
         return;
      }

      windows.push_back(std::move(out));
      tracklets.push_back(tracklet);
      while(tracklets.size() > 2) tracklets.pop_front();
      ++n_windows;
      if(stats != nullptr)
         stats->add_window_ms(ms_between(t_start, steady_clock::now()));

      vector<FrameTimeline> window_timelines;
      window_timelines.reserve(frames.size());
      for(const auto& rec : frames) window_timelines.push_back(rec.timeline);
      timelines.push_back(std::move(window_timelines));
      while(timelines.size() > 2) timelines.pop_front();
      frames.clear();

      if(observer) observer(windows.back());

      if(windows.size() >= 2)
         publish_revised(
             windows[windows.size() - 2],
             &windows.back(),
             calc_id_remap(provisional, windows[windows.size() - 2].seqs),
             timelines.front());
      publish_tracked(windows.back(), timelines.back());
   }

   // -------------------------------------------------------------- on-record
   //
   void on_record(FrameRecord&& record)
   {
      if(is_failed) return;
      const int frame_no = record.frame_no();
      if(frame_no != next_frame or frame_no >= p.end_frame) {
         fail(frame_no / K,
              format("frame {} arrived where frame {} was expected (end {})",
                     frame_no,
                     next_frame,
                     p.end_frame));
         return;
      }
      ++next_frame;
      frames.push_back(std::move(record));
      if(frame_no == window_last(frame_no)) close_window();
   }

   // ----------------------------------------------------------------- on-end
   // The last (maybe partial) window, and its revision
   void on_end()
   {
      if(!is_failed and !frames.empty()) close_window();
      if(!is_failed and !windows.empty())
         publish_revised(windows.back(), nullptr, {}, timelines.back());
   }

   // -------------------------------------------------------------------- run
   //
   void run() noexcept
   {
      while(true) {
         std::optional<FrameRecord> record;
         try {
            record = queue.pop();
         } catch(std::exception& e) {
            LOG_ERR(format("exception waiting for a frame: {}", e.what()));
         }
         if(!record) break;
         try {
            on_record(std::move(*record));
         } catch(std::exception& e) {
            fail(record->frame_no() / K, format("exception: {}", e.what()));
         }
      }

      try {
         on_end();
      } catch(std::exception& e) {
         fail(next_frame / K, format("exception: {}", e.what()));
      }
   }

   // ------------------------------------------------------------------- stop
   //
   void stop() noexcept
   {
      queue.close();
      try {
         if(thread.joinable()) thread.join();
      } catch(std::exception& e) {
         LOG_ERR(format("exception joining the tracker thread: {}", e.what()));
      }
   }
};

// ---------------------------------------------------------------- Construction
//
OnlineTracker::OnlineTracker(shared_ptr<const SceneDescription> scene,
                             const Params& p,
                             publish_fn publish,
                             window_fn window,
                             observer_fn observer,
                             RealtimeStats* stats)
    : pimpl_(make_unique<Pimpl>(std::move(scene),
                                p,
                                std::move(publish),
                                std::move(window),
                                std::move(observer),
                                stats))
{}

OnlineTracker::~OnlineTracker() { pimpl_->stop(); }

// --------------------------------------------------------------------- Getters
//
const OnlineTracker::Params& OnlineTracker::params() const noexcept
{
   return pimpl_->p;
}

int OnlineTracker::frames_per_window() const noexcept { return pimpl_->K; }

int OnlineTracker::delta_t() const noexcept { return pimpl_->stream.delta_t(); }

bool OnlineTracker::has_error() const noexcept { return pimpl_->has_error; }

size_t OnlineTracker::queue_size() const noexcept
{
   return pimpl_->queue.size();
}

int OnlineTracker::n_windows() const noexcept { return pimpl_->n_windows; }

int OnlineTracker::n_frames() const noexcept { return pimpl_->n_frames_pushed; }

const TrackerResult& OnlineTracker::raw_result() const noexcept
{
   return pimpl_->raw;
}

const FowlkesResult& OnlineTracker::presmooth_result() const noexcept
{
   return pimpl_->presmooth;
}

const FowlkesResult& OnlineTracker::final_result() const noexcept
{
   return pimpl_->final_result;
}

// ----------------------------------------------------------------------- start
//
bool OnlineTracker::start() noexcept
{
   try {
      pimpl_->thread = std::thread([this]() { pimpl_->run(); });
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to start the tracker thread: {}", e.what()));
   }
   return false;
}

// ------------------------------------------------------------------------ push
//
bool OnlineTracker::push(FrameRecord&& record) noexcept
{
   try {
      return pimpl_->queue.push(std::move(record));
   } catch(std::exception& e) {
      LOG_ERR(format("exception queueing frame {} for the tracker: {}",
                     record.frame_no(),
                     e.what()));
   }
   return false;
}

// ---------------------------------------------------------------------- finish
//
void OnlineTracker::finish() noexcept
{
   auto& P = *pimpl_;
   if(P.is_finished) return;
   P.is_finished = true;
   P.stop();

   P.n_frames_pushed = P.next_frame - P.p.start_frame;
   if(P.windows.empty()) return;

   vector<const Tracks*> windows;
   windows.reserve(P.windows.size());
   for(const auto& tracks : P.windows) windows.push_back(&tracks);
   P.raw          = make_tracker_result(windows,
                               P.p.start_frame,
                               P.n_frames_pushed,
                               P.header.top_left,
                               P.header.hist_sz,
                               P.p.frame_duration);
   P.final_result = finalise_tracks(
       P.raw, P.p.post_process, P.p.fowlkes_params, &P.presmooth);
}

} // namespace perceive::pipeline::realtime
