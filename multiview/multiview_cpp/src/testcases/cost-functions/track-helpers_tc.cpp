
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/track-ops.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/movie/render-tracks.hpp"
#include "perceive/utils/file-system.hpp"

static const bool feedback = false;

// namespace perceive
// {
// using namespace perceive::detail;

// struct TracksHelper
// {
//    Tracks::Params p;

//    shared_ptr<const SceneDescription> scene_desc;
//    vector<LocalizationData> loc_dat;
//    vector<const LocalizationData*> loc_ptrs;
//    vector<Tracklet> tracklets;
//    vector<Tracks> tracks0;
//    vector<Tracks> tracks;
//    real hist_sz             = 0.0;
//    int max_frames_per_slice = 0;
//    int w                    = -1;
//    int h                    = -1;
//    int end_frame_no         = -1;
//    vector<Vector2> zone     = {{0.0, -0.5},
//                            {0.0, 0.0},
//                            {2.0, 0.0},
//                            {2.0, -0.5},
//                            {4.0, -0.5},
//                            {4.0, -2.5},
//                            {-3.0, -2.5},
//                            {-3.0, -0.5}};

//    int total_n_frames() noexcept { return loc_dat.size(); }

//    void reset_tracks0()
//    {
//       tracks0.clear();
//       tracks0.resize(tracklets.size());
//       std::transform(cbegin(tracklets),
//                      cend(tracklets),
//                      begin(tracks0),
//                      [&](const auto& tracklet) {
//                         return track_ops::detail::tracklets_to_tracks(
//                             p, tracklet, 0);
//                      });
//       int max_id = 0;
//       for(auto& x : tracks0)
//          for(auto& tt : x.tracks) tt.id = max_id++;
//    }

//    void run_tracklets(const Tracklet::Params& p, const bool feedback)
//    {
//       const int total_frames = loc_dat.size();
//       for(auto i = 0u; i < tracklets.size(); ++i) {
//          const int start_frame = i * max_frames_per_slice;
//          const int n_frames
//              = std::min(start_frame + max_frames_per_slice, total_frames)
//                - start_frame;

//          tracklets[i] = Tracklet::execute(
//              *scene_desc,
//              p,
//              start_frame,
//              n_frames,
//              max_frames_per_slice,
//              feedback,
//              [&](int frame_no) { return get_loc_dat_ptr(frame_no); },
//              []() { return false; });

//          if(feedback) { cout << tracklets[i].to_string() << endl << endl; }
//       }
//    }

//    float log_prob_dist(float dist)
//    {
//       const float frame_duration = 1.0f / 15.0f;
//       const float speed_median   = 1.4f; // 2.0 meters per second
//       const float speed_stddev   = 1.0f; // 2.0 meters per second
//       const float mps            = dist / frame_duration;
//       const float z              = (mps - speed_median) / speed_stddev;
//       return -log(phi_function(-z));
//    }

//    real vertex_weight(const TrackPoint& u)
//    {
//       const unsigned t = u.t;
//       Expects(t < loc_ptrs.size());
//       const auto loc = loc_ptrs[t];
//       Expects(loc != nullptr);
//       Expects(loc->in_bounds(u.x, u.y));
//       return loc->hist_cost(u.x, u.y);
//    }

//    real edge_weight(const TrackPoint& u, const TrackPoint& v)
//    {
//       const auto dist
//           = (Vector2(u.x, u.y) - Vector2(v.x, v.y)).norm() * hist_sz;
//       return log_prob_dist(dist) + vertex_weight(v);
//    }

//    bool in_bounds(const Point2& u) { return loc_ptrs[0]->in_bounds(u.x, u.y);
//    }

//    const Tracks* get_track0_ptr(int frameno)
//    {
//       auto idx = frameno / max_frames_per_slice;
//       Expects(unsigned(idx) < tracks0.size());
//       return &tracks0[idx];
//    }

//    const Tracks* get_track_ptr(int frameno)
//    {
//       auto idx = frameno / max_frames_per_slice;
//       Expects(unsigned(idx) < tracks.size());
//       return &tracks[idx];
//    }

//    const Tracklet* get_tracklet_ptr(int frameno)
//    {
//       auto idx = frameno / max_frames_per_slice;
//       return (unsigned(idx) < tracklets.size()) ? &tracklets[idx] : nullptr;
//    }

//    const LocalizationData* get_loc_dat_ptr(int frameno)
//    {
//       return (unsigned(frameno) >= loc_dat.size()) ? nullptr
//                                                    : &loc_dat[frameno];
//    }

//    void init(shared_ptr<const SceneDescription> scene_desc, const string
//    fname)
//    {
//       this->scene_desc = scene_desc;

//       FILE* fp = fopen(fname.data(), "rb");
//       if(!fp) FATAL(format("failed to open '{:s}' for reading", fname));

//       LocalizationDataEnvelope loc_envelope = load_localization_data(fp);
//       loc_dat                               = loc_envelope.loc_data;
//       // tracklets                             = load_tracklets_data(fp);
//       Expects(tracklets.size() > 0);

//       loc_ptrs.resize(loc_dat.size());
//       for(auto i = 0u; i < loc_dat.size(); ++i) loc_ptrs[i] = &loc_dat[i];
//       Expects(loc_ptrs.size() > 0);

//       hist_sz              = loc_ptrs[0]->hist_sz;
//       max_frames_per_slice = tracklets[0].max_frames_per_tracklet;
//       w                    = loc_ptrs[0]->loc_hist.width;
//       h                    = loc_ptrs[0]->loc_hist.height;
//       end_frame_no         = loc_dat.size();

//       auto get_track_ptr = [this](int frameno) -> const Tracks* {
//          return this->get_track_ptr(frameno);
//       };

//       auto get_tracklet_ptr = [this](int frameno) -> const Tracklet* {
//          return this->get_tracklet_ptr(frameno);
//       };

//       auto get_loc_dat_ptr = [this](int frameno) -> const LocalizationData* {
//          return this->get_loc_dat_ptr(frameno);
//       };

//       // Calculate tracks0
//       auto make_tracks = [&](unsigned idx) -> Tracks {
//          return ::perceive::calc_track(p,
//                                        scene_desc->frame_duration(),
//                                        idx * max_frames_per_slice,
//                                        (idx > 0 ? &tracks[idx - 1] :
//                                        nullptr), get_tracklet_ptr,
//                                        get_loc_dat_ptr,
//                                        []() { return false; });
//       };

//       auto push_tracks = [&]() {
//          const unsigned idx = tracks.size();
//          WARN(format("tracks {}/{}", idx, tracklets.size()));
//          tracks.push_back(make_tracks(idx));
//          cout << tracklets[idx].to_string() << endl;
//          cout << tracks.back().to_string() << endl;
//       };
//    }

//    static void run_full_movie(shared_ptr<const SceneDescription> scene_desc,
//                               const string_view fname,
//                               const string_view out_movie_fname)
//    {
//       TracksHelper th;
//       th.init(scene_desc, fname.data());
//       th.reset_tracks0();
//       const Tracklet::Params p;
//       th.run_tracklets(p, true);

//       auto get_tracklet_ptr = [&th](int frameno) -> const Tracklet* {
//          return th.get_tracklet_ptr(frameno);
//       };

//       auto get_loc_dat_ptr = [&th](int frameno) -> const LocalizationData* {
//          return th.get_loc_dat_ptr(frameno);
//       };

//       const int feedback_level   = 0;
//       const int frames_per_epoch = th.tracks0[0].max_frames_per_track;
//       vector<Tracks> tracks(th.tracklets.size());
//       auto now = tick();
//       for(auto i = 0u; i < th.tracklets.size(); ++i) {
//          Tracks* previous_track = (i > 0) ? &tracks[i - 1] : nullptr;
//          const int frame_no     = i * frames_per_epoch;

//          tracks[i] = ::perceive::track_ops::detail::calc_tracks_ops_method(
//              th.p,
//              scene_desc->frame_duration(),
//              get_tracklet_ptr(frame_no),
//              previous_track,
//              get_loc_dat_ptr,
//              []() { return false; },
//              feedback_level);
//       }
//       // WARN(format("th, calc tracks = {}s", tock(now)));

//       if(false)
//          for(const auto& tt : tracks) { cout << tt.to_string() << endl; }

//       auto get_track_ptr = [&](int frameno) -> const Tracks* {
//          const auto idx = frameno / th.max_frames_per_slice;
//          return (unsigned(idx) < tracks.size()) ? &tracks[idx] : nullptr;
//       };

//       if(scene_desc == nullptr) {
//          // render_tracks_movie(th.p,
//          //                     scene_desc->frame_duration(),
//          //                     th.zone,
//          //                     tracks,
//          //                     get_loc_dat_ptr,
//          //                     out_movie_fname.data());
//          FATAL("kBAM!");
//       } else {
//          int start_frame = 0;
//          int n_frames    = th.total_n_frames();
//          const auto tmpd = make_temp_directory("/tmp/multiview-");
//          pipeline::DebugMovieParams params(1024, 768);
//          params.render_tracks_on_raw_video = true;
//          params.render_openpose            = true;
//          params.p3ds_on_localization       = true;

//          WARN(format("Still have to fix target fps, and frame duration"));
//          const real target_fps     = 15.0;
//          const real frame_duration = 1.0 / 15.0;

//          make_debug_movie(scene_desc.get(),
//                           params,
//                           start_frame,
//                           n_frames,
//                           get_loc_dat_ptr,
//                           get_tracklet_ptr,
//                           get_track_ptr,
//                           nullptr,
//                           tmpd,
//                           out_movie_fname);
//          remove_all(tmpd);
//       }
//    }
// };

// // ------------------------------------------------------------------
// run-a-test
// //
// static void run_a_test(const string manifest_fname,
//                        const string th_fname,
//                        const string mp4_fname)
// {
//    if(!is_hermes_build()) return;

//    bool has_error = false;

//    if(!is_regular_file(th_fname)) {
//       WARN(format("th-fname '{:s}', not found.", th_fname));
//       has_error = true;
//    }

//    if(!is_regular_file(manifest_fname)) {
//       WARN(format("manifest-fname '{:s}', not found.", manifest_fname));
//       has_error = true;
//    }

//    if(has_error) {
//       WARN(format("aborting testcase because of previous errors: testcase "
//                   "should be inactive."));
//       CATCH_REQUIRE(false);
//    }

//    SceneDescription::InitOptions opts;
//    // opts.target_fps = ;
//    const auto scene_desc = load_scene_from_manifest(manifest_fname, opts);
//    Expects(scene_desc);

//    TracksHelper th;
//    th.run_full_movie(scene_desc, th_fname, mp4_fname);
// }

// // ------------------------------------------------------------------- Test
// Case
// //

// const std::string& perceive_data_dir() noexcept;

// // CATCH_TEST_CASE("TrackHelpers", "[track-helpers]")
// // {
// //    // ---------------------------------------------------------
// track-helpers
// //    B7 CATCH_SECTION("track-helpers_B7")
// //    {
// //       const auto perceive_data_dir =

// //       const auto manifest_fname
// //           = format("{:s}/computer-vision/experiments/test-runs/"
// //                    "envista_short_2019-03-14_14-11-04/manifest.json",
// //                    perceive_data_dir);

// //       const auto th_fname = format(
// // "%s/computer-vision/test-data/track-helpers_tc/loc-tracklet_B7.data",
// //           perceive_data_dir);

// //       if(multiview_trace_mode())
// //          run_a_test(manifest_fname, th_fname, "/tmp/B7.mp4"s);
// //    }
// // }

// } // namespace perceive
