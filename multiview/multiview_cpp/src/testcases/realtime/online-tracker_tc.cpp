
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes.hpp"
#include "perceive/geometry/point-cloud.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/realtime/online-tracker.hpp"
#include "perceive/pipeline/realtime/track-stream.hpp"
#include "perceive/utils/file-system.hpp"

#include <deque>
#include <map>

namespace perceive::pipeline::realtime
{
using tracks::Node;
using tracks::NodeSequence;

namespace tracker_test
{
   constexpr int k_n_sensors = 2;
   constexpr int k_n_cameras = 1;
   constexpr int k_hist_w    = 40;
   constexpr int k_hist_h    = 30;

   // ------------------------------------------------------------- make-scene
   // A scene with sensors and cameras, and no calibration: enough for
   // `Tracklet::execute`, which reads only the counts
   static shared_ptr<const SceneDescription> make_scene()
   {
      auto scene = make_shared<SceneDescription>();
      for(int i = 0; i < k_n_sensors; ++i)
         scene->sensor_ids.push_back(format("S{}", i));
      for(int i = 0; i < k_n_cameras; ++i)
         scene->filenames.push_back(format("C{}", i));
      return scene;
   }

   // ----------------------------------------------------------- make-record
   //
   static FrameRecord make_record(int frame_no)
   {
      const LocalizationData::Params p;
      LocalizationData like;
      like.hist_sz = 0.1;
      like.bounds  = AABB(-2.0, -1.0, 2.0, 2.0);
      like.loc_min = float(p.loc_min());
      like.loc_max = float(p.loc_max());
      like.loc_hist.resize(k_hist_w, k_hist_h);
      like.p2ds.resize(k_n_sensors);

      FrameRecord record;
      record.timeline.frame_no  = frame_no;
      record.timeline.t_arrival = std::chrono::steady_clock::now();
      record.loc_ptr            = make_shared<const LocalizationData>(
          LocalizationData::make_empty(like, p, frame_no));
      for(int i = 0; i < k_n_sensors; ++i)
         record.labs.push_back(make_shared<const LABImage>());
      for(int i = 0; i < k_n_cameras; ++i)
         record.point_clouds.push_back(make_shared<const PointCloud>());
      return record;
   }

   static FrameRecord make_drop(int frame_no)
   {
      FrameTimeline timeline;
      timeline.frame_no = frame_no;
      return FrameRecord::make_dropped(timeline, DropReason::NO_FREE_SLOT);
   }

   // -------------------------------------------------------------- make-node
   //
   static Node make_node(int t, float x, float y, int detection_base)
   {
      Node node;
      node.set_t(t);
      node.set_xy(Vector2f(x, y));
      node.set_gaze(0.25f);
      node.set_prob_fp(0.05f);
      node.set_pose_scores(vector<float>(size_t(n_pose_annotations()), 0.5f));
      node.set_p2ds({P2dAddress(t, 0, detection_base)});
      return node;
   }

   // -------------------------------------------------------------- the world
   // Three people. A (id 0) walks through every frame; C (id 3) is there for
   // frames [0, 5]; B is there for [10, 25], with id 7 until the window after
   // its first one renames it to 5. A revision moves every node by 0.25.
   static vector<NodeSequence> world_seqs(int start, int n, bool is_revised)
   {
      std::map<int, vector<Node>> nodes;
      const float dx = is_revised ? 0.25f : 0.0f;
      for(int t = start; t < start + n; ++t) {
         nodes[0].push_back(make_node(t, 5.0f + 0.5f * t + dx, 10.0f, 0));
         if(t <= 5) nodes[3].push_back(make_node(t, 30.0f + dx, 5.0f, 1));
         if(t >= 10 and t <= 25) {
            const bool is_renamed = is_revised or start >= 20;
            const int id          = is_renamed ? 5 : 7;
            nodes[id].push_back(make_node(t, 20.0f + dx, 0.2f * t, 2));
         }
      }
      vector<NodeSequence> out;
      for(auto& [id, seq_nodes] : nodes)
         out.emplace_back(id, std::move(seq_nodes));
      return out;
   }

   // ----------------------------------------------------------- world-window
   // A window function in the shape of `finalize_result_`
   static Tracks world_window(const Tracklet* tracklet,
                              const Tracklet* prev_tracklet,
                              Tracks* prev)
   {
      Tracks out;
      out.seqs = world_seqs(tracklet->start_frame, tracklet->n_frames, false);
      if(prev != nullptr)
         prev->seqs = world_seqs(prev->start_frame, prev->n_frames, true);
      out.max_id               = 7;
      out.w                    = k_hist_w;
      out.h                    = k_hist_h;
      out.hist_sz              = 0.1;
      out.aabb                 = AABB(-2.0, -1.0, 2.0, 2.0);
      out.start_frame          = tracklet->start_frame;
      out.n_frames             = tracklet->n_frames;
      out.max_frames_per_track = 10;
      return out;
   }

   // ----------------------------------------------------------- make-params
   //
   static OnlineTracker::Params make_params(int start, int end)
   {
      OnlineTracker::Params p;
      p.start_frame    = start;
      p.end_frame      = end;
      p.frame_duration = 1.0 / 15.0;
      return p;
   }

   // --------------------------------------------------------------- Captured
   // The messages a tracker published, and the windows its function saw
   struct Captured
   {
      std::mutex padlock;
      vector<TrackMessage> messages;
      struct Call
      {
         int start_frame        = -1;
         int n_frames           = 0;
         bool has_prev          = false;
         bool has_prev_tracklet = false;
         vector<int> loc_frames; // `loc_ptr->frame_no` of every frame
         vector<bool> has_p2ds;
         vector<const LABImage*> labs; // sensor 0 of every frame
      };
      vector<Call> calls;

      OnlineTracker::publish_fn publisher()
      {
         return [this](TrackMessage&& msg) {
            std::lock_guard<std::mutex> lock(padlock);
            messages.push_back(std::move(msg));
         };
      }

      OnlineTracker::window_fn window(OnlineTracker::window_fn inner)
      {
         return [this, inner](const Tracklet* tracklet,
                              const Tracklet* prev_tracklet,
                              Tracks* prev) {
            Call call;
            call.start_frame       = tracklet->start_frame;
            call.n_frames          = tracklet->n_frames;
            call.has_prev          = prev != nullptr;
            call.has_prev_tracklet = prev_tracklet != nullptr;
            for(const auto& fd : tracklet->frames) {
               call.loc_frames.push_back(fd.loc_ptr->frame_no);
               size_t n_p2ds = 0;
               for(const auto& sensor_p2ds : fd.loc_ptr->p2ds)
                  n_p2ds += sensor_p2ds.size();
               call.has_p2ds.push_back(n_p2ds > 0);
               call.labs.push_back(fd.sensor_labs.at(0).get());
            }
            calls.push_back(call);
            return inner(tracklet, prev_tracklet, prev);
         };
      }

      template<typename T> vector<T> all() const
      {
         vector<T> out;
         for(const auto& msg : messages)
            if(std::holds_alternative<T>(msg)) out.push_back(std::get<T>(msg));
         return out;
      }
   };

   // ------------------------------------------------------------------- run
   // Pushes `records` through a tracker, and finishes it
   static void run(OnlineTracker& tracker, vector<FrameRecord>&& records)
   {
      CATCH_REQUIRE(tracker.start());
      for(auto& record : records) tracker.push(std::move(record));
      tracker.finish();
   }

   // ---------------------------------------------------- oracle-concatenate
   // The concatenation of `TrackCalculator::get_tracker_result` before it
   // moved into `make_tracker_result`
   static vector<NodeSequence>
   oracle_concatenate(const std::deque<Tracks>& windows)
   {
      int max_id = -1;
      for(const auto& w : windows)
         for(const auto& seq : w.seqs) max_id = std::max(max_id, seq.id());
      vector<NodeSequence> seqs(size_t(max_id + 1));
      vector<bool> marks(seqs.size(), false);
      for(const auto& w : windows) {
         for(const auto& seq : w.seqs) {
            if(!marks[size_t(seq.id())]) {
               seqs[size_t(seq.id())]  = seq;
               marks[size_t(seq.id())] = true;
            } else {
               seqs[size_t(seq.id())].append(seq);
            }
         }
      }
      auto ii = std::stable_partition(
          begin(seqs), end(seqs), [](const auto& o) { return o.size() > 0; });
      seqs.erase(ii, end(seqs));
      return seqs;
   }

   // -------------------------------------------------------------- same-seqs
   //
   static bool same_seqs(const vector<NodeSequence>& a,
                         const vector<NodeSequence>& b)
   {
      if(a.size() != b.size()) return false;
      for(size_t i = 0; i < a.size(); ++i) {
         if(a[i].id() != b[i].id() or a[i].size() != b[i].size()) return false;
         for(size_t j = 0; j < a[i].size(); ++j) {
            const auto& u = a[i].nodes()[j];
            const auto& v = b[i].nodes()[j];
            if(u.t() != v.t() or u.xy() != v.xy()) return false;
         }
      }
      return true;
   }

   // ------------------------------------------------------- make-sinusoid
   //
   static NodeSequence make_sinusoid(int id, int t0, int t1)
   {
      vector<Node> nodes;
      for(int t = t0; t < t1; ++t) {
         Node node = make_node(t,
                               float(10.0 + 3.0 * std::sin(0.3 * t)),
                               float(5.0 + 2.0 * std::cos(0.2 * t)),
                               0);
         node.set_gaze(float(0.1 * t));
         nodes.push_back(node);
      }
      return NodeSequence(id, std::move(nodes));
   }

   static Tracks make_tracks(vector<NodeSequence> seqs)
   {
      Tracks out;
      out.seqs = std::move(seqs);
      return out;
   }

} // namespace tracker_test

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("OnlineTracker", "[realtime_tracker]")
{
   using namespace tracker_test;

   CATCH_SECTION("protocol")
   {
      Captured cap;
      OnlineTracker tracker(make_scene(),
                            make_params(0, 30),
                            cap.publisher(),
                            cap.window(world_window));
      vector<FrameRecord> records;
      for(int t = 0; t < 30; ++t) records.push_back(make_record(t));
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(tracker.n_windows() == 3);
      CATCH_REQUIRE(tracker.n_frames() == 30);
      CATCH_REQUIRE(cap.calls.size() == 3);
      for(const auto& call : cap.calls)
         CATCH_REQUIRE(call.has_prev == call.has_prev_tracklet);
      CATCH_REQUIRE(!cap.calls[0].has_prev);
      CATCH_REQUIRE(cap.calls[1].has_prev);

      // Every frame once per revision, revised after tracked
      const auto tracks = cap.all<TracksMessage>();
      std::map<std::pair<int, int>, int> counts;
      for(const auto& msg : tracks) counts[{msg.frame, msg.revision}]++;
      CATCH_REQUIRE(counts.size() == 60);
      for(const auto& [key, count] : counts) CATCH_REQUIRE(count == 1);

      // Revision 1 of window 0 holds the revised positions, in metres
      for(const auto& msg : tracks) {
         if(msg.frame != 3) continue;
         const auto& a = msg.tracks.at(0);
         CATCH_REQUIRE(a.id == 0);
         const real hist_x = 5.0 + 0.5 * 3 + (msg.revision == 1 ? 0.25 : 0.0);
         CATCH_REQUIRE(std::fabs(a.x - (-2.0 + 0.1 * hist_x)) < 1e-6);
         CATCH_REQUIRE(msg.status
                       == (msg.revision == 1 ? "revised" : "tracked"));
      }

      // B is 7 when tracked in window 1, and 5 once revised
      bool has_remap = false;
      for(const auto& msg : tracks) {
         if(msg.frame != 15) continue;
         const auto& b = msg.tracks.back();
         CATCH_REQUIRE(b.id == (msg.revision == 0 ? 7 : 5));
         if(msg.revision == 1)
            has_remap = (msg.id_remap == vector<std::pair<int, int>>{{7, 5}});
      }
      CATCH_REQUIRE(has_remap);

      // C ends at frame 5, and that is published when window 1 runs
      const auto ends = cap.all<TrackEndMessage>();
      CATCH_REQUIRE(ends.size() == 3); // C, then A and B at the end
      CATCH_REQUIRE(ends[0].id == 3);
      CATCH_REQUIRE(ends[0].last_t == 5);
      size_t end_pos   = 0;
      size_t frame_pos = 0;
      for(size_t i = 0; i < cap.messages.size(); ++i) {
         const auto& msg = cap.messages[i];
         if(std::holds_alternative<TrackEndMessage>(msg) and end_pos == 0)
            end_pos = i;
         if(std::holds_alternative<TracksMessage>(msg)
            and std::get<TracksMessage>(msg).frame == 20)
            frame_pos = i;
      }
      CATCH_REQUIRE(end_pos < frame_pos);

      // Smoothed: every point of every track is final once
      std::map<std::pair<int, int>, int> finals;
      for(const auto& msg : cap.all<SmoothedMessage>())
         for(const auto& pt : msg.points)
            if(pt.t <= msg.final_upto_t) finals[{msg.id, pt.t}]++;
      CATCH_REQUIRE(finals.size() == size_t(30 + 6 + 16));
      for(const auto& [key, count] : finals) CATCH_REQUIRE(count == 1);

      // The result: the batch concatenation of the windows
      std::deque<Tracks> windows;
      for(int k = 0; k < 3; ++k) {
         Tracklet tracklet;
         tracklet.start_frame = 10 * k;
         tracklet.n_frames    = 10;
         Tracklet prev_tracklet;
         windows.push_back(world_window(&tracklet,
                                        (k == 0) ? nullptr : &prev_tracklet,
                                        (k == 0) ? nullptr : &windows.back()));
      }
      const auto& raw = tracker.raw_result();
      CATCH_REQUIRE(raw.start_frame == 0);
      CATCH_REQUIRE(raw.n_frames == 30);
      CATCH_REQUIRE(raw.hist_w == k_hist_w);
      CATCH_REQUIRE(same_seqs(raw.seqs, oracle_concatenate(windows)));
      CATCH_REQUIRE(raw.seqs.size() == 3);
   }

   CATCH_SECTION("gap-is-empty-with-borrowed-images")
   {
      Captured cap;
      OnlineTracker tracker(make_scene(),
                            make_params(0, 20),
                            cap.publisher(),
                            cap.window(world_window));
      vector<FrameRecord> records;
      for(int t = 0; t < 20; ++t)
         records.push_back((t == 13) ? make_drop(t) : make_record(t));
      const auto* lab12 = records[12].labs[0].get();
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(cap.calls.size() == 2);
      const auto& call = cap.calls[1];
      CATCH_REQUIRE(call.loc_frames[3] == 13);
      CATCH_REQUIRE(call.labs[3] == lab12);
      CATCH_REQUIRE(!call.has_p2ds[3]);
   }

   CATCH_SECTION("leading-gap")
   {
      Captured cap;
      OnlineTracker tracker(make_scene(),
                            make_params(0, 20),
                            cap.publisher(),
                            cap.window(world_window));
      vector<FrameRecord> records;
      for(int t = 0; t < 20; ++t)
         records.push_back((t < 7) ? make_drop(t) : make_record(t));
      const auto* lab7 = records[7].labs[0].get();
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(cap.calls.size() == 2);
      const auto& call = cap.calls[0];
      CATCH_REQUIRE(call.start_frame == 0);
      CATCH_REQUIRE(!call.has_prev);
      for(int t = 0; t < 7; ++t) {
         CATCH_REQUIRE(call.loc_frames[size_t(t)] == t);
         CATCH_REQUIRE(call.labs[size_t(t)] == lab7);
      }
   }

   CATCH_SECTION("first-window-without-a-good-frame")
   {
      Captured cap;
      OnlineTracker tracker(make_scene(),
                            make_params(0, 30),
                            cap.publisher(),
                            cap.window(world_window));
      vector<FrameRecord> records;
      for(int t = 0; t < 30; ++t)
         records.push_back((t < 10) ? make_drop(t) : make_record(t));
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(cap.calls.size() == 2);
      CATCH_REQUIRE(cap.calls[0].start_frame == 10);
      CATCH_REQUIRE(!cap.calls[0].has_prev);
      CATCH_REQUIRE(cap.calls[1].has_prev);
      for(const auto& msg : cap.all<TracksMessage>())
         CATCH_REQUIRE(msg.frame >= 10);
      CATCH_REQUIRE(tracker.raw_result().start_frame == 0);
   }

   CATCH_SECTION("later-all-gap-window")
   {
      Captured cap;
      OnlineTracker tracker(make_scene(),
                            make_params(0, 30),
                            cap.publisher(),
                            cap.window(world_window));
      vector<FrameRecord> records;
      for(int t = 0; t < 30; ++t)
         records.push_back((t >= 10 and t < 20) ? make_drop(t)
                                                : make_record(t));
      const auto* lab9 = records[9].labs[0].get();
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(cap.calls.size() == 3);
      const auto& call = cap.calls[1];
      CATCH_REQUIRE(call.start_frame == 10);
      CATCH_REQUIRE(call.has_prev);
      for(size_t i = 0; i < 10; ++i) {
         CATCH_REQUIRE(call.loc_frames[i] == 10 + int(i));
         CATCH_REQUIRE(call.labs[i] == lab9);
      }
   }

   CATCH_SECTION("validation")
   {
      { // A missing frame
         Captured cap;
         OnlineTracker tracker(make_scene(),
                               make_params(0, 20),
                               cap.publisher(),
                               cap.window(world_window));
         vector<FrameRecord> records;
         for(int t = 0; t < 20; ++t)
            if(t != 5) records.push_back(make_record(t));
         run(tracker, std::move(records));
         CATCH_REQUIRE(tracker.has_error());
         CATCH_REQUIRE(cap.calls.empty());
         CATCH_REQUIRE(cap.all<WindowErrorMessage>().size() == 1);
         CATCH_REQUIRE(cap.all<TracksMessage>().empty());
      }

      { // A window whose result does not cover its frames
         Captured cap;
         auto short_window = [](const Tracklet* tracklet,
                                const Tracklet* prev_tracklet,
                                Tracks* prev) {
            auto out     = world_window(tracklet, prev_tracklet, prev);
            out.n_frames = 5;
            return out;
         };
         OnlineTracker tracker(make_scene(),
                               make_params(0, 30),
                               cap.publisher(),
                               cap.window(short_window));
         vector<FrameRecord> records;
         for(int t = 0; t < 30; ++t) records.push_back(make_record(t));
         run(tracker, std::move(records));
         CATCH_REQUIRE(tracker.has_error());
         CATCH_REQUIRE(cap.calls.size() == 1);
         const auto errors = cap.all<WindowErrorMessage>();
         CATCH_REQUIRE(errors.size() == 1);
         CATCH_REQUIRE(errors[0].window == 1);
      }
   }

   CATCH_SECTION("smoothing")
   {
      fowlkes::Params fp;
      KzFilter filter(int(fp.smooth_kz_filter_m), int(fp.smooth_kz_filter_k));
      TrackStream stream(int(fp.smooth_kz_filter_m),
                         int(fp.smooth_kz_filter_k));
      CATCH_REQUIRE(stream.delta_t() == 16);

      Track batch = node_sequence_to_track(make_sinusoid(4, 100, 160));
      smooth_track(filter, batch);
      auto batch_point = [&](int t) -> const perceive::TrackPoint& {
         return batch.path.at(size_t(t - 100));
      };

      // 40 nodes, and the track goes on
      const auto next  = make_tracks({make_sinusoid(4, 140, 141)});
      const auto first = stream.update(
          make_tracks({make_sinusoid(4, 100, 140)}), &next, 139);
      CATCH_REQUIRE(first.size() == 1);
      CATCH_REQUIRE(first[0].final_upto_t == 139 - 16);
      CATCH_REQUIRE(!first[0].is_ended);
      for(const auto& tp : first[0].track.path) {
         if(tp.t > first[0].final_upto_t) continue;
         CATCH_REQUIRE(tp.x == batch_point(tp.t).x);
         CATCH_REQUIRE(tp.y == batch_point(tp.t).y);
         CATCH_REQUIRE(tp.gaze_direction == batch_point(tp.t).gaze_direction);
      }

      // 20 more, and the track ends: every point equals the batch's
      const auto second = stream.update(
          make_tracks({make_sinusoid(4, 140, 160)}), nullptr, 159);
      CATCH_REQUIRE(second.size() == 1);
      CATCH_REQUIRE(second[0].is_ended);
      CATCH_REQUIRE(second[0].final_upto_t == 159);
      CATCH_REQUIRE(second[0].track.path.front().t == 124);
      CATCH_REQUIRE(second[0].track.path.back().t == 159);
      for(const auto& tp : second[0].track.path) {
         CATCH_REQUIRE(tp.x == batch_point(tp.t).x);
         CATCH_REQUIRE(tp.y == batch_point(tp.t).y);
         CATCH_REQUIRE(tp.gaze_direction == batch_point(tp.t).gaze_direction);
      }

      // A sequence that does not continue its track is skipped
      const auto third = stream.update(
          make_tracks({make_sinusoid(4, 170, 175)}), nullptr, 174);
      CATCH_REQUIRE(third.empty());
      CATCH_REQUIRE(stream.sequence(4)->nodes().back().t() == 159);
   }

   CATCH_SECTION("gap-only-window-through-the-window-code")
   {
      const char* data_dir        = getenv("MULTIVIEW_TESTCASE_DATA");
      const string manifest_fname = format(
          "{}/testcases/007_site03_sparse_tracks/manifest.json",
          (data_dir != nullptr) ? string(data_dir)
                                : format("{}/multiview-data", getenv("HOME")));
      if(!is_regular_file(manifest_fname)) {
         WARN(format("testcase '{}' not found, skipping", manifest_fname));
         return;
      }

      auto scene = make_shared<SceneDescription>();
      SceneDescription::InitOptions opts;
      opts.no_stereo            = true;
      opts.optional_search_path = dirname(manifest_fname);
      try {
         scene->init_from_json(parse_json(file_get_contents(manifest_fname)),
                               opts);
      } catch(std::exception& e) {
         WARN(format(
             "failed to load '{}', skipping: {}", manifest_fname, e.what()));
         return;
      }

      // Frames with the scene's sensors and no detections
      const LocalizationData::Params lp;
      LocalizationData like;
      like.hist_sz = 0.1;
      like.bounds  = scene->scene_info.hist_bounds;
      if(!like.bounds.is_finite()) like.bounds = AABB(-5.0, -5.0, 5.0, 5.0);
      like.loc_min = float(lp.loc_min());
      like.loc_max = float(lp.loc_max());
      like.loc_hist.resize(
          unsigned(std::ceil(like.bounds.width() / like.hist_sz)),
          unsigned(std::ceil(like.bounds.height() / like.hist_sz)));
      like.p2ds.resize(size_t(scene->n_sensors()));

      auto make_gap_record = [&](int t) {
         FrameRecord record = make_drop(t);
         record.loc_ptr     = make_shared<const LocalizationData>(
             LocalizationData::make_empty(like, lp, t));
         for(int i = 0; i < scene->n_sensors(); ++i)
            record.labs.push_back(make_shared<const LABImage>());
         for(int i = 0; i < scene->n_cameras(); ++i)
            record.point_clouds.push_back(make_shared<const PointCloud>());
         record.drop_reason = DropReason::NONE; // a frame with no detections
         return record;
      };

      Captured cap;
      auto p           = make_params(0, 20);
      p.frame_duration = scene->frame_duration();
      p.loc_params     = lp;
      OnlineTracker tracker(scene, p, cap.publisher());
      vector<FrameRecord> records;
      for(int t = 0; t < 20; ++t) records.push_back(make_gap_record(t));
      run(tracker, std::move(records));

      CATCH_REQUIRE(!tracker.has_error());
      CATCH_REQUIRE(tracker.n_windows() == 2);
      CATCH_REQUIRE(tracker.raw_result().seqs.empty());
      CATCH_REQUIRE(tracker.final_result().tracks.empty());
      CATCH_REQUIRE(cap.all<TracksMessage>().size() == 40);
   }
}

} // namespace perceive::pipeline::realtime
