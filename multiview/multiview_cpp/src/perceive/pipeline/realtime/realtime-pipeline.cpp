
#include "realtime-pipeline.hpp"

#include "bounded-queue.hpp"
#include "frame-slot-pool.hpp"
#include "latency-stats.hpp"
#include "online-tracker.hpp"
#include "paced-frame-source.hpp"
#include "provisional-localizer.hpp"
#include "publisher.hpp"
#include "reorder-buffer.hpp"
#include "track-messages.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/optimization/kz-filter.hpp"
#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/detail/task-timings.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/pipeline/pipeline-input.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/pipeline/test-output.hpp"
#include "perceive/utils/file-system.hpp"

#include <csignal>
#include <thread>
#include <unistd.h>

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;
using std::chrono::steady_clock;
using time_point = steady_clock::time_point;

// Set by the first SIGINT; a second one exits at once
static std::atomic<bool> interrupted_ = false;

// ---------------------------------------------------------------- on-interrupt
//
static void on_interrupt(int) noexcept
{
   if(interrupted_.exchange(true)) std::_Exit(EXIT_FAILURE);
}

// ------------------------------------------------------------------ ms-between
//
static double ms_between(time_point a, time_point b) noexcept
{
   return std::chrono::duration<double, std::milli>(b - a).count();
}

// ----------------------------------------------------------------- timeline-of
//
static FrameTimeline timeline_of(const SourceFrame& frame) noexcept
{
   FrameTimeline timeline;
   timeline.frame_no  = frame.frame_no;
   timeline.timestamp = frame.timestamp;
   timeline.t_due     = frame.t_due;
   timeline.t_arrival = frame.t_arrival;
   return timeline;
}

// ----------------------------------------------------------------- RealtimeRun
// One run: the stages in data-flow order, and the threads between them.
// Members are destroyed in reverse order, so the slots go before what their
// callback touches, and the scene goes last.
//
struct RealtimeRun
{
   // --------------------------------------------------------------- Parameters
   CliArgs config;
   RealtimeParams rp;
   MovieResults::Params in_params;
   unique_ptr<MovieResults> movie;
   shared_ptr<const SceneDescription> scene;
   int start_frame       = 0;
   int end_frame         = 0; // exclusive
   real rate             = 0.0;
   bool drop_newest      = false;
   real cluster_diameter = 0.4;
   HeaderMessage header;
   real startup_s = 0.0; // scene, slots and warm-up
   real warm_up_s = 0.0;

   // ------------------------------------------------------------------- Output
   FILE* out_fp           = nullptr;
   string output_filename = ""s; // pipeline-output.json, once written

   // ------------------------------------------------------------------- Stages
   unique_ptr<RealtimeStats> stats;
   LatencyStats dag_ms;  // dispatch => result
   LatencyStats wait_ms; // arrival => dispatch
   unique_ptr<PacedFrameSource> source;
   unique_ptr<BoundedQueue<FrameRecord>> loc_q;
   unique_ptr<ReorderBuffer> reorder;
   unique_ptr<ProvisionalLabeller> labeller;
   unique_ptr<FrameSlotPool> slots;
   OnlineTracker::Params tracker_params;
   unique_ptr<OnlineTracker> tracker;
   unique_ptr<Publisher> publisher;
   std::thread dispatcher;
   std::thread reorder_thread;
   std::atomic<bool> stop_requested  = false;
   std::atomic<bool> dispatcher_done = false;
   bool is_shut_down                 = false;
   time_point t_start                = {};
   time_point t_end                  = {};

   // ------------------------------------------------------ Reorder thread only
   bool is_header_checked = false;
   int n_frames           = 0; // frame numbers written, processed or dropped
   int n_dropped          = 0;
   int n_processed        = 0;

   RealtimeRun(const CliArgs& in_config, const RealtimeParams& in_rp)
       : config(in_config)
       , rp(in_rp)
   {}
   RealtimeRun(const RealtimeRun&)            = delete;
   RealtimeRun& operator=(const RealtimeRun&) = delete;
   ~RealtimeRun()
   {
      shutdown();
      if(publisher) publisher->close();
      if(out_fp != nullptr) fclose(out_fp);
   }

   bool init() noexcept(false);
   void start() noexcept(false);
   void wait() noexcept;
   void shutdown() noexcept;
   bool finish() noexcept;

   void publish(TrackMessage&& msg) noexcept;
   void check_header(const LocalizationData& loc) noexcept;
   StatsMessage make_stats(time_point now) noexcept;
   void emit(const FrameRecord& record) noexcept;
   void emit_and_track(FrameRecord&& record) noexcept;
   void on_slot_done(FrameRecord&& record) noexcept;
   void push_drop(int frame_no, DropReason reason) noexcept;
   void run_dispatcher() noexcept;
   void run_reorder() noexcept;
   Json::Value summary_json() const noexcept;
   bool write_tracks() noexcept;
};

// ------------------------------------------------------------------------ init
//
bool RealtimeRun::init() noexcept(false)
{
   const auto now = tick();

   // ----------------------- The scene, and the parameters `pipeline` would use
   if(!in_params.init(config, false)) return false;
   if(!in_params.load_manifest()) return false;
   scene = in_params.scene_desc;
   Expects(scene != nullptr);
   movie = make_unique<MovieResults>(in_params);

   auto& master = *movie->frame_results;
   master.set_feedback(false);
   {
      auto p                = master.copy_sensor_images.params();
      p.still_sample_period = rp.still_sample_period;
      p.still_max_frames    = rp.still_max_frames;
      master.copy_sensor_images.set_params(p);
   }

   const auto& pose_params = master.pose_skeleton_init.params().op_params;
   if(!pose_params.replay_dir.empty() and multiview_openpose_disabled()) {
      LOG_ERR(format("poses would not be replayed from '{}': "
                     "MULTIVIEW_OPENPOSE_DISABLED is set",
                     pose_params.replay_dir));
      return false;
   }
   const auto& precision = pose_params.hp_params.precision;
   if(precision != "fp32"s and precision != "fp16"s) {
      LOG_ERR(format("hp_params.precision must be 'fp32' or 'fp16', not '{}'",
                     precision));
      return false;
   }

   // --------------------------------------------- Frames, pacing, and dropping
   start_frame = config.start_frame_no;
   end_frame   = (config.n_frames_to_process < 0)
                     ? scene->n_frames()
                     : std::min(scene->n_frames(),
                              start_frame + config.n_frames_to_process);
   if(start_frame < 0 or start_frame >= end_frame) {
      LOG_ERR(format("no frames to process: start = {}, end = {}, scene has {}",
                     start_frame,
                     end_frame,
                     scene->n_frames()));
      return false;
   }
   rate        = rp.rate(scene->scene_fps);
   drop_newest = rp.drops_newest(scene->scene_fps);

   const auto pipeline_input = movie->params().params.to_json();
   cluster_diameter          = 2.0 * lookup_person_radius(pipeline_input);

   // -------------------- params.json, as `pipeline` writes it, plus "realtime"
   {
      Json::Value o    = in_params.params.to_json();
      o["config"]      = config.to_json();
      o["task_params"] = movie->task_params();
      o["realtime"]    = rp.to_json();
      file_put_contents(format("{}/params.json", config.outdir),
                        perceive::str(o));
   }

   // --------------------------------------------------------------- The header
   {
      const auto& fp = in_params.params.fowlkes_params;
      const KzFilter filter(int(fp.smooth_kz_filter_m),
                            int(fp.smooth_kz_filter_k));
      // As `floor_hist` does; with no area it would FATAL on a pool thread
      AABB bounds = scene->scene_info.hist_bounds;
      if(!bounds.is_finite()) bounds = AABB{-5.0, -5.0, 5.0, 5.0};
      if(!(bounds.area() > 0.0)) {
         LOG_ERR(format("the scene's histogram bounds have no area: {}",
                        str(bounds)));
         return false;
      }
      header.scene       = scene->scene_info.scene_key;
      header.n_cameras   = scene->n_cameras();
      header.fps         = scene->scene_fps;
      const real hist_sz = real(master.floor_hist.params().hist_params.hist_sz);
      header.hist_sz     = hist_sz;
      header.top_left_x  = bounds.left;
      header.top_left_y  = bounds.top;
      header.w = int(ceil((bounds.right - bounds.left) / hist_sz) + 1e-9);
      header.h = int(ceil((bounds.bottom - bounds.top) / hist_sz) + 1e-9);
      header.start_frame = start_frame;
      header.frames_per_window
          = int(movie->tracklet_task.params()
                    .tracklet_params.max_frames_per_tracklet);
      header.delta_t = filter.delta_t();
      header.version = k_version;
   }

   // -------------------------------------- The tracker: what the batch would
   // use
   {
      auto& tp           = tracker_params;
      tp.tracklet_params = movie->tracklet_task.params().tracklet_params;
      tp.tracks_params   = movie->tracks_task.params().tracks_params;
      tp.post_process    = movie->params().post_process_params;
      tp.fowlkes_params  = in_params.params.fowlkes_params;
      tp.loc_params      = master.localization.params().localization_params;
      tp.start_frame     = start_frame;
      tp.end_frame       = end_frame;
      tp.frame_duration  = scene->frame_duration();
      tp.outdir          = config.outdir;

      const int K = int(tp.tracklet_params.max_frames_per_tracklet);
      if(K <= 0) {
         LOG_ERR(format("max_frames_per_tracklet must be positive"));
         return false;
      }
      if(start_frame % K != 0
         or (end_frame % K != 0 and end_frame != scene->n_frames()))
         WARN(format("frames [{}, {}) do not start and end on tracking windows "
                     "of {} frames: the first or last window is shorter than "
                     "the batch's",
                     start_frame,
                     end_frame,
                     K));
   }

   // --------------------------------------------------------------- The output
   if(rp.ndjson_path == "-"s) {
      const int fd = dup(STDOUT_FILENO);
      out_fp       = (fd >= 0) ? fdopen(fd, "w") : nullptr;
   } else {
      out_fp = fopen(rp.ndjson_path.c_str(), "w");
   }
   if(out_fp == nullptr) {
      LOG_ERR(format("failed to open '{}' for writing", rp.ndjson_path));
      return false;
   }

   // ------------------------------------------------------------ The publisher
   {
      vector<Publisher::sink_fn> sinks;
      sinks.push_back([fp = out_fp](const string& line) {
         return fputs(line.c_str(), fp) >= 0 and fflush(fp) == 0;
      });
      Publisher::Params pp;
      pp.stats_period_s = rp.stats_period_s;
      publisher         = make_unique<Publisher>(
          pp, std::move(sinks), [this](time_point now) {
             return make_stats(now);
          });
   }

   // --------------------------------------------------------- The paced source
   {
      SceneManifest manifest;
      read(manifest, config.manifest_params);
      const auto map = make_video_frame_map(*scene, manifest);

      PacedFrameSource::Params sp;
      sp.start_frame      = start_frame;
      sp.end_frame        = end_frame;
      sp.scene_fps        = scene->scene_fps;
      sp.timestamp0       = scene->timestamp0;
      sp.rate_fps         = rate;
      sp.ring_size        = rp.ring_size_or_default();
      sp.drop_when_behind = drop_newest;
      sp.split            = [scene = scene](vector<cv::Mat>&& raw, int k) {
         return scene->split_raw_images(std::move(raw), k);
      };
      source
          = make_unique<PacedFrameSource>(make_video_camera_readers(map), sp);
   }

   // ----------------------------------------- Queues, slots, and their warm-up
   loc_q   = make_unique<BoundedQueue<FrameRecord>>(rp.n_slots + 8);
   reorder = make_unique<ReorderBuffer>(
       start_frame,
       drop_newest ? std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::duration<double>(rp.missing_timeout_s))
                   : std::chrono::nanoseconds(0));
   labeller = make_unique<ProvisionalLabeller>(
       rp.match_radius, 2 * std::max(1, header.frames_per_window));
   slots = make_unique<FrameSlotPool>(
       master, rp.n_slots, [this](FrameRecord&& record) {
          on_slot_done(std::move(record));
       });

   const auto warm_tick = tick();
   if(!slots->warm_up()) {
      LOG_ERR(format("failed to compute the slots' camera geometry"));
      return false;
   }
   warm_up_s = tock(warm_tick);
   startup_s = tock(now);
   INFO(format("realtime: {} slots warmed up in {}s (start-up {}s); frames "
               "[{}, {}) at {} fps, drop {}",
               rp.n_slots,
               warm_up_s,
               startup_s,
               start_frame,
               end_frame,
               rate,
               (drop_newest ? "newest" : "none")));
   return true;
}

// ----------------------------------------------------------------------- start
//
void RealtimeRun::start() noexcept(false)
{
   t_start = steady_clock::now();
   stats   = make_unique<RealtimeStats>(t_start);
   publish(header);
   tracker = make_unique<OnlineTracker>(
       scene,
       tracker_params,
       [this](TrackMessage&& msg) { publish(std::move(msg)); },
       OnlineTracker::window_fn{},
       [this](const Tracks& newest) {
          labeller->set_tracks(live_tracks(newest));
       },
       stats.get());
   if(!publisher->start())
      throw std::runtime_error("failed to start the publisher thread");
   if(!tracker->start())
      throw std::runtime_error("failed to start the tracker thread");
   source->start();
   dispatcher     = std::thread([this]() { run_dispatcher(); });
   reorder_thread = std::thread([this]() { run_reorder(); });
}

// ------------------------------------------------------------------------ wait
// Until the dispatcher has taken every frame, SIGINT, or `max_seconds`
//
void RealtimeRun::wait() noexcept
{
   while(!dispatcher_done) {
      std::this_thread::sleep_for(50ms);
      if(interrupted_) {
         INFO(format("realtime: interrupted, stopping"));
         break;
      }
      if(tracker->has_error()) {
         LOG_ERR(format("realtime: the tracker failed, stopping"));
         break;
      }
      const real elapsed = ms_between(t_start, steady_clock::now()) / 1000.0;
      if(rp.max_seconds > 0.0 and elapsed >= rp.max_seconds) {
         INFO(format("realtime: stopping after {}s", elapsed));
         break;
      }
   }
}

// -------------------------------------------------------------------- shutdown
// In data-flow order, so that every frame a slot took is written out
//
void RealtimeRun::shutdown() noexcept
{
   if(is_shut_down) return;
   is_shut_down   = true;
   stop_requested = true;
   if(source) source->stop();
   if(dispatcher.joinable()) dispatcher.join();
   if(slots) slots->drain();
   if(loc_q) loc_q->close();
   if(reorder_thread.joinable()) reorder_thread.join();
   if(tracker) tracker->finish();
}

// --------------------------------------------------------------------- publish
//
void RealtimeRun::publish(TrackMessage&& msg) noexcept
{
   const char* type = message_type(msg);
   if(!publisher->publish(std::move(msg)))
      LOG_ERR(format("a '{}' message was published after the end", type));
}

// ---------------------------------------------------------------- check-header
// The header was written before any frame; warn if a frame's histogram
// disagrees with it
//
void RealtimeRun::check_header(const LocalizationData& loc) noexcept
{
   is_header_checked = true;
   if(int(loc.loc_hist.width) != header.w
      or int(loc.loc_hist.height) != header.h
      or loc.bounds.left != header.top_left_x
      or loc.bounds.top != header.top_left_y)
      WARN(format("frame {}'s histogram ({}x{} at [{}, {}]) differs from the "
                  "header's ({}x{} at [{}, {}])",
                  loc.frame_no,
                  loc.loc_hist.width,
                  loc.loc_hist.height,
                  loc.bounds.left,
                  loc.bounds.top,
                  header.w,
                  header.h,
                  header.top_left_x,
                  header.top_left_y));
}

// ------------------------------------------------------------------ make-stats
// The `stats` message (publisher or main thread)
//
StatsMessage RealtimeRun::make_stats(time_point now) noexcept
{
   stats->set_queue_depth(RealtimeStats::Queue::LOC, loc_q->size());
   stats->set_queue_depth(RealtimeStats::Queue::TRACKER, tracker->queue_size());
   stats->set_queue_depth(RealtimeStats::Queue::OUT, publisher->queue_size());
   stats->set_slots_busy(slots->n_busy());
   StatsMessage msg;
   msg.stats = stats->to_json(now);
   return msg;
}

// ------------------------------------------------------------------------ emit
// One frame, in frame order (reorder thread)
//
void RealtimeRun::emit(const FrameRecord& record) noexcept
{
   ++n_frames;

   if(record.is_dropped()) {
      ++n_dropped;
      stats->count_drop(record.drop_reason);
      DroppedMessage msg;
      msg.frame  = record.frame_no();
      msg.reason = str(record.drop_reason);
      publish(std::move(msg));
      return;
   }

   ++n_processed;
   const auto& timeline = record.timeline;
   const auto& loc      = *record.loc_ptr;
   if(!is_header_checked) check_header(loc);
   auto positions = provisional_positions(*scene, loc, cluster_diameter);
   labeller->label(record.frame_no(), positions);

   FrameMessage msg;
   msg.frame     = record.frame_no();
   msg.timestamp = str(timeline.timestamp);
   for(const auto& sensor_p2ds : loc.p2ds)
      msg.n_p2ds += int(sensor_p2ds.size());
   for(const auto& o : positions) {
      FramePosition pos;
      pos.x            = o.X.x;
      pos.y            = o.X.y;
      pos.n_detections = o.n_detections;
      pos.score        = real(o.score);
      pos.prob_fp      = real(o.prob_fp);
      pos.pid          = o.pid;
      msg.positions.push_back(pos);
   }

   const auto now = steady_clock::now();
   msg.latency_ms = ms_between(timeline.t_arrival, now);
   stats->count_frame(RealtimeStats::Stage::PROVISIONAL, now);
   stats->add_latency(RealtimeStats::Latency::PROVISIONAL, msg.latency_ms);
   if(rp.publish_provisional) publish(std::move(msg));
   dag_ms.add(ms_between(timeline.t_dispatch, timeline.t_dag_done));
   wait_ms.add(ms_between(timeline.t_arrival, timeline.t_dispatch));
}

// -------------------------------------------------------------- emit-and-track
// One frame, in frame order, to the stream and then to the tracker, which
// blocks while the tracker is a window behind (reorder thread)
//
void RealtimeRun::emit_and_track(FrameRecord&& record) noexcept
{
   emit(record);
   tracker->push(std::move(record));
}

// ---------------------------------------------------------------- on-slot-done
// A slot's result (pool thread). Admission keeps room in `loc_q` for every
// busy slot, so the push only fails if the queue was closed.
//
void RealtimeRun::on_slot_done(FrameRecord&& record) noexcept
{
   stats->count_frame(RealtimeStats::Stage::DAG, record.timeline.t_dag_done);
   const int frame_no = record.frame_no();
   if(!loc_q->try_push(std::move(record))) {
      LOG_ERR(format("no room for frame {}; it is lost", frame_no));
      push_drop(frame_no, DropReason::DAG_ERROR);
   }
}

// ------------------------------------------------------------------- push-drop
//
void RealtimeRun::push_drop(int frame_no, DropReason reason) noexcept
{
   reorder->push_drop(frame_no, frame_no, reason, steady_clock::now());
}

// -------------------------------------------------------------- run-dispatcher
// Hands frames to idle slots. With `drop_newest`, frames that wait while a
// newer one arrives are dropped; otherwise every frame waits for a slot.
// A frame is admitted only when `loc_q` has room for it and for every busy
// slot's result.
//
void RealtimeRun::run_dispatcher() noexcept
{
   auto take_source_frame = [&](const shared_ptr<const SourceFrame>& frame) {
      if(frame->is_dropped()) {
         push_drop(frame->frame_no, frame->drop_reason);
         return false;
      }
      stats->count_frame(RealtimeStats::Stage::SOURCE, frame->t_arrival);
      return true;
   };

   if(!drop_newest) {
      while(!stop_requested) {
         const auto frame = source->next(50ms);
         if(frame == nullptr) {
            if(source->is_finished()) break;
            continue;
         }
         if(!take_source_frame(frame)) continue;
         if(!loc_q->wait_for_room(slots->n_busy() + 1)) break;
         if(!slots->dispatch(
                frame, timeline_of(*frame), FrameSlotPool::k_wait_forever))
            push_drop(frame->frame_no, DropReason::DAG_ERROR);
      }
   } else {
      shared_ptr<const SourceFrame> pending = nullptr;
      DropReason pending_reason             = DropReason::NO_FREE_SLOT;
      while(!stop_requested) {
         bool has_waited = false;
         if(pending != nullptr) {
            if(!loc_q->has_room(slots->n_busy() + 1)) {
               pending_reason = DropReason::TRACKER_BEHIND;
            } else {
               has_waited = true;
               if(slots->dispatch(pending, timeline_of(*pending), 5ms))
                  pending = nullptr;
               else
                  pending_reason = DropReason::NO_FREE_SLOT;
            }
         }

         const auto timeout = has_waited ? 0ms : (pending ? 5ms : 20ms);
         const auto frames  = source->take_all(timeout);
         for(const auto& frame : frames) {
            if(!take_source_frame(frame)) continue;
            if(pending != nullptr) push_drop(pending->frame_no, pending_reason);
            pending        = frame;
            pending_reason = DropReason::NO_FREE_SLOT;
         }

         if(pending == nullptr and frames.empty() and source->is_finished())
            break;
      }
      if(pending != nullptr) push_drop(pending->frame_no, pending_reason);
   }

   dispatcher_done = true;
}

// ----------------------------------------------------------------- run-reorder
// Writes frames in order as their results come in
//
void RealtimeRun::run_reorder() noexcept
{
   while(true) {
      auto record    = loc_q->pop_for(10ms);
      const auto now = steady_clock::now();
      if(record) reorder->push(std::move(*record), now);
      while(auto ready = reorder->pop_ready(now))
         emit_and_track(std::move(*ready));
      if(!record and loc_q->is_closed() and loc_q->size() == 0) break;
   }

   for(auto& record : reorder->flush()) emit_and_track(std::move(record));
   t_end = steady_clock::now();
}

// ---------------------------------------------------------------- summary-json
//
Json::Value RealtimeRun::summary_json() const noexcept
{
   const auto src     = source->stats();
   const real seconds = ms_between(t_start, t_end) / 1000.0;

   Json::Value o{Json::objectValue};
   o["n_slots"]            = rp.n_slots;
   o["max_busy"]           = slots->max_busy();
   o["replay_rate"]        = rate;
   o["drop"]               = drop_newest ? "newest" : "none";
   o["start_frame"]        = start_frame;
   o["end_frame"]          = end_frame;
   o["frames"]             = n_frames;
   o["processed"]          = n_processed;
   o["dropped"]            = n_dropped;
   o["run_seconds"]        = seconds;
   o["processed_fps"]      = (seconds > 0.0) ? n_processed / seconds : 0.0;
   o["startup_seconds"]    = startup_s;
   o["warm_up_seconds"]    = warm_up_s;
   o["dag_ms"]             = dag_ms.to_json();
   o["dispatch_wait_ms"]   = wait_ms.to_json();
   o["n_late"]             = Json::UInt64(reorder->n_late());
   o["n_expired"]          = Json::UInt64(reorder->n_expired());
   o["source_produced"]    = Json::UInt64(src.produced);
   o["source_behind"]      = Json::UInt64(src.skipped_behind);
   o["source_decode_errs"] = Json::UInt64(src.decode_errors);
   o["source_evicted"]     = Json::UInt64(src.evicted);
   o["stats"]              = stats->to_json(t_end);
   o["tracker_windows"]    = tracker->n_windows();
   o["tracker_frames"]     = tracker->n_frames();
   o["tracker_error"]      = tracker->has_error();
   o["raw_tracks"]         = int(tracker->raw_result().seqs.size());
   o["kept_tracks"]        = int(tracker->final_result().tracks.size());
   o["output"]             = output_filename;

   Json::Value timings{Json::arrayValue};
   for(unsigned i = 0; i < slots->n_slots(); ++i) {
      Json::Value slot{Json::objectValue};
      for(const auto& info : slots->slot_timings(i).timings()) {
         if(info.count == 0) continue;
         slot[info.name] = 1000.0 * info.average_duration();
      }
      timings.append(slot);
   }
   o["slot_mean_ms"] = timings;
   return o;
}

// ---------------------------------------------------------------- write-tracks
// After `shutdown`: `pipeline-output.json` as `pipeline` writes it, the same
// before smoothing (`pipeline-output-presmooth.json`), and the testcase
// scores. False on error.
//
bool RealtimeRun::write_tracks() noexcept
{
   if(tracker->n_windows() == 0) {
      WARN(format("no tracking window ran; there is no pipeline-output.json"));
      return true;
   }

   try {
      const auto& p = movie->params();
      PipelineOutput outdata;
      outdata.config        = p.config;
      outdata.params        = p.params.to_json();
      outdata.track_results = tracker->presmooth_result().to_json();
      save(outdata, format("{}/pipeline-output-presmooth.json", config.outdir));
      outdata.track_results = tracker->final_result().to_json();
      save(outdata, p.config.pipeline_output_filename);
      output_filename = p.config.pipeline_output_filename;

      if(p.config.run_pipeline_testcase) {
         const auto& calc = tracker->final_result();
         const auto& gt   = p.ground_truth;
         const auto result
             = compare_tracker_output(unsigned(start_frame),
                                      unsigned(calc.frames),
                                      gt.tracks,
                                      gt.frame_duration,
                                      calc.tracks,
                                      calc.frame_duration,
                                      calc.bounds(),
                                      calc.hist_sz,
                                      lookup_person_radius(outdata.params),
                                      to_radians(45.0));
         INFO("testcase results:");
         cout << str(result) << endl;
      }
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("exception writing the tracks: {}", e.what()));
   }
   return false;
}

// ---------------------------------------------------------------------- finish
// After `shutdown`: the tracks, the last lines, and the summary files. False
// if the tracker failed or the tracks could not be written.
//
bool RealtimeRun::finish() noexcept
{
   const bool has_tracks = write_tracks();

   publish(make_stats(t_end));
   EndMessage end_msg;
   end_msg.frames  = n_frames;
   end_msg.dropped = n_dropped;
   end_msg.output  = output_filename;
   publish(std::move(end_msg));
   publisher->close();

   try {
      const auto summary = summary_json();
      file_put_contents(format("{}/realtime-stats.json", config.outdir),
                        perceive::str(summary));
      INFO(format(
          "realtime: {} frames, {} processed, {} dropped, {}s; "
          "provisional latency p50 {} ms, p95 {} ms",
          n_frames,
          n_processed,
          n_dropped,
          summary["run_seconds"].asDouble(),
          summary["stats"]["latency_ms"]["provisional"]["p50"].asDouble(),
          summary["stats"]["latency_ms"]["provisional"]["p95"].asDouble()));
      INFO(format("realtime: {} tracking windows; latency p50/p95 tracked "
                  "{}/{} ms, revised {}/{} ms, final {}/{} ms",
                  tracker->n_windows(),
                  summary["stats"]["latency_ms"]["tracked"]["p50"].asDouble(),
                  summary["stats"]["latency_ms"]["tracked"]["p95"].asDouble(),
                  summary["stats"]["latency_ms"]["revised"]["p50"].asDouble(),
                  summary["stats"]["latency_ms"]["revised"]["p95"].asDouble(),
                  summary["stats"]["latency_ms"]["final"]["p50"].asDouble(),
                  summary["stats"]["latency_ms"]["final"]["p95"].asDouble()));

      if(config.generate_timings)
         for(unsigned i = 0; i < slots->n_slots(); ++i)
            render_timings(slots->slot_timings(i))
                .save(
                    format("{}/realtime-timings-slot{}.png", config.outdir, i));
   } catch(std::exception& e) {
      LOG_ERR(format("exception writing the run's summary: {}", e.what()));
   }

   return has_tracks and !tracker->has_error();
}

// --------------------------------------------------------- print-frame-mapping
//
bool print_frame_mapping(const CliArgs& config, std::ostream& out) noexcept
{
   try {
      MovieResults::Params in_params;
      if(!in_params.init(config, false)) return false;
      if(!in_params.load_manifest()) return false;
      Expects(in_params.scene_desc != nullptr);
      const auto& scene = *in_params.scene_desc;

      SceneManifest manifest;
      read(manifest, config.manifest_params);
      const auto map = make_video_frame_map(scene, manifest);

      out << format("# scene frame => movie frame; scene fps {}, movie fps {}",
                    map.scene_fps,
                    map.movie_fps)
          << endl;
      for(size_t k = 0; k < map.frame_mapping.size(); ++k)
         out << format("{} {}", k, map.frame_mapping[k]) << endl;
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to make the frame mapping: {}", e.what()));
   }
   return false;
}

// ---------------------------------------------------------------- run-realtime
//
bool run_realtime(const CliArgs& config, const RealtimeParams& rp) noexcept
{
   interrupted_          = false;
   const auto old_signal = std::signal(SIGINT, on_interrupt);

   bool success = false;
   try {
      RealtimeRun run(config, rp);
      if(run.init()) {
         run.start();
         run.wait();
         run.shutdown();
         success = run.finish();
      }
   } catch(std::exception& e) {
      LOG_ERR(format("realtime: {}", e.what()));
   }

   std::signal(SIGINT, old_signal);
   if(success) INFO(format("realtime-execute done."));
   return success;
}

} // namespace perceive::pipeline::realtime
