
#include "debug-movie.hpp"

#include "draw-floor-grid.hpp"
#include "ffmpeg.hpp"
#include "movie-layout.hpp"
#include "movie-utils.hpp"
#include "render-tracks.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/localization/calc-prob-p2d-false-positive.hpp"
#include "perceive/cost-functions/tracks/ops.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/pipeline/movie-results.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

namespace perceive::movie
{
// --------------------------------------------------------- delete_files_in_dir
//
static size_t delete_files_in_dir(const string_view dir, const string_view ext)
{
   const auto ret = remove_all(
       dir, [&ext](const string_view s) { return file_ext(s) == ext; });
   return size_t(ret);
}

// -------------------------------------------------------------- make cam fname
//
string make_cam_fname(const string_view directory,
                      const int frame_no,
                      const int cam_ind,
                      const int sensor_num) noexcept
{
   return format(
       "{}/{:06d}_cam{}_{}.png", directory, frame_no, cam_ind, sensor_num);
}

// -------------------------------------------------------------- make loc fname
//
string make_loc_fname(const string_view directory, const int frame_no) noexcept
{
   return format("{}/{:6d}_loc.png", directory, frame_no);
}

string make_tracklet_fname(const string_view directory,
                           const int frame_no) noexcept
{
   return format("{}/{:6d}_tracklets.png", directory, frame_no);
}

string make_tracks_fname(const string_view directory,
                         const int frame_no) noexcept
{
   return format("{}/{:6d}_tracks.png", directory, frame_no);
}

string make_composite_fname(const string_view directory,
                            const int frame_no) noexcept
{
   return format("{}/{:6d}_composite.png", directory, frame_no);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// ------------------------------------------------------------ make debug movie
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

DebugMovieParams::DebugMovieParams(int w_, int h_)
    : w(w_)
    , h(h_)
{}

DebugMovieParams::DebugMovieParams(const pipeline::CliArgs& args)
{
   w                          = args.output_video_width;
   h                          = args.output_video_height;
   render_tracks_on_raw_video = args.output_video_render_tracks;
   render_pose                = args.output_video_render_pose;
   p2ds_on_localization       = args.output_video_p2ds_on_localization;
   blur_faces                 = args.output_video_blur_faces;
   render_ground_truth_only   = args.output_video_ground_truth_only;
   render_ground_truth        = false;
   // = args.run_pipeline_testcase && !args.output_video_ground_truth_only;
}

RenderSensorTimings&
RenderSensorTimings::operator+=(const RenderSensorTimings& o) noexcept
{
   total_p2ds += o.total_p2ds;
   blur_faces_s += o.blur_faces_s;
   render_grid_s += o.render_grid_s;
   render_p2d_hists_s += o.render_p2d_hists_s;
   render_heights_line_s += o.render_heights_line_s;
   render_p2d_cylinders_s += o.render_p2d_cylinders_s;
   render_pose_skeleton_s += o.render_pose_skeleton_s;
   render_tracks_on_raw_video_s += o.render_tracks_on_raw_video_s;
   return *this;
}

static std::pair<vector<std::pair<Point2, string>>, RenderSensorTimings>
render_sensor(const SceneDescription* scene_desc,
              const DebugMovieParams& params,
              const LocalizationData::Params& loc_params,
              const LocalizationData* loc_dat,
              const vector<Track>* tracks_ptr,
              const vector<Track>* gt_tracks_ptr,
              const int frame_no,
              const int sensor_no,
              const ARGBImage& floor_grid,
              ARGBImage& argb)
{
   // Timers
   RenderSensorTimings timings = {};

   Expects(loc_dat != nullptr);
   Expects(scene_desc != nullptr);
   Expects(size_t(sensor_no) < loc_dat->p2ds.size());

   const auto& dcam  = scene_desc->dcam(sensor_no);
   const auto& et    = scene_desc->sensor_transforms[size_t(sensor_no)];
   const auto& cu    = scene_desc->cu(sensor_no);
   const auto& infos = loc_dat->p2ds[size_t(sensor_no)];

   const real hist_sz  = loc_dat->hist_sz;
   const auto& bounds  = loc_dat->bounds;
   const auto top_left = loc_dat->bounds.top_left();

   // Track the total number of p2ds in this frame
   timings.total_p2ds += infos.size();

   // Project function for the current `dcam`
   auto project = [&](const auto& X) { return project_to_distorted(dcam, X); };

   auto proj_to_hist = [&](const Vector3& X) -> Vector2 {
      return FloorHistogram::project_to_histogram(X, top_left, hist_sz);
   };

   auto unproject_cell_xyz = [&](const Point2& xy) -> Vector3 {
      return FloorHistogram::unproject_hist_xyz(xy, top_left, hist_sz);
   };

   vector<std::pair<Point2, string>> pose_labels;

   // -- (*) -- Blur faces
   if(params.blur_faces) {
      const auto now = tick();
      blur_faces(argb,
                 frame_no,
                 false,
                 unsigned(infos.size()),
                 [&infos](unsigned idx) -> const Skeleton2D* {
                    return infos[idx].p2d_ptr.get();
                 });
      timings.blur_faces_s += tock(now);
   }

   // -- (*) -- Render Grid
   if(params.render_grid_on_raw_video) {
      const auto now   = tick();
      const auto& grid = floor_grid;
      Expects(grid.height == argb.height);
      Expects(grid.width == argb.width);
      const uint32_t mask_colour = 0xff000000u;
      for(auto y = 0u; y < grid.height; ++y)
         for(auto x = 0u; x < grid.width; ++x)
            if(grid(x, y) != mask_colour) argb(x, y) = grid(x, y);
      timings.render_grid_s += tock(now);
   }

   // -- (*) -- Render p2d hists
   if(params.render_p2d_hist_on_raw_video) {
      const auto now = tick();

      int info_counter = 0;
      for(const auto& info : infos) {
         const auto scores = calc_prob_p2d_fp_scores(
             loc_params, *info.p2d_ptr, &info.hist, bounds, hist_sz, dcam);

         const auto& p2d = *info.p2d_ptr;
         const auto sum  = sparse_histcell_sum(scores.cloud_weighted_xy);
         const auto hsv0 = rgb_to_hsv(kolour_to_vector3(k_blue));

         for(const auto& cell : scores.cloud_weighted_xy) {
            const auto X   = unproject_cell_xyz(cell.xy);
            const auto wgt = cell.count / sum;
            if(!(wgt >= 0.0f && wgt <= 1.0f)) {
               // TRACE(format("found wgt = {}", wgt));
               continue;
            }
            const auto intensity = logish_prob(cell.count / sum);
            auto hsv             = hsv0;
            hsv.y                = real(intensity);

            const uint32_t kolour = vector3_to_kolour(hsv_to_rgb(hsv));
            AABB pos = AABB(X.x, X.y, X.x - hist_sz, X.y - hist_sz);
            draw_floor_cell_in_situ(argb, pos, cu, et, k_cyan, kolour);
         }

         ++info_counter;
      }

      timings.render_p2d_hists_s += tock(now);
   }

   // -- (*) -- Render Plausible Heights Line
   if(params.render_plausible_heights_line) {
      const auto now = tick();
      // Draw the "plausible heights" line
      const auto delta = 0.2;
      for(const auto& info : infos) {
         const auto& p2d = *info.p2d_ptr;
         bool first      = true;
         for(auto hgt = 1.40; hgt <= 2.100001; hgt += delta) {
            const auto a = project(p2d.realize_cylinder(dcam, hgt).Cy.X);
            const auto b
                = project(p2d.realize_cylinder(dcam, hgt + delta).Cy.X);
            plot_line_AA(a, b, argb.bounds(), [&](int x, int y, float a) {
               if(!argb.in_bounds(x, y)) return;
               argb(x, y) = blend(k_red, argb(x, y), a);
            });
            if(first) draw_cross_thick(argb, to_pt2(a), k_cyan, 4);
            draw_cross_thick(argb, to_pt2(b), k_cyan, 4);
            first = false;
         }
      }
      timings.render_heights_line_s += tock(now);
   }

   auto get_track_id = [&](const auto& p3d) -> int {
      if(tracks_ptr == nullptr) return -1;
      const Vector2 x = proj_to_hist(to_vec3(p3d.Xs_centre()));
      for(const auto& tt : *tracks_ptr)
         for(const auto& tp : tt.path)
            if((x - to_vec2(tp.xy())).norm() * hist_sz < params.person_radius)
               return tt.id;
      return -1;
   };

   // -- (*) -- Render "best" p2d cylinders
   if(params.render_p2d_cylinders) {
      // TRACE(format("params.render_p2d_cylinders = {}",
      //              str(params.render_p2d_cylinders)));

      const auto now = tick();
      int counter    = 0;
      for(const auto& info : infos) {
         const auto& p2d = *info.p2d_ptr;
         const auto& p3d = p2d.best_3d_result();

         const uint32_t kolour = colour_set_4(unsigned(info.id));

         //
         const bool render_it
             = params.render_false_positives || get_track_id(p3d) >= 0;

         if(render_it) {
            plot_skeleton_3d(
                dcam, p3d, fNAN, argb.bounds(), [&](int x, int y, float alpha) {
                   if(argb.in_bounds(x, y)) {
                      argb(x, y) = blend(kolour, argb(x, y), alpha);
                   }
                });
         }

         ++counter;
      }
      timings.render_p2d_cylinders_s += tock(now);
   }

   // -- (*) -- Render poses
   if(params.render_pose) {
      // TRACE(format("params.render_pose = {}", str(params.render_pose)));

      const auto now = tick();
      pose_labels
          = movie::render_poses(argb,
                                sensor_no,
                                unsigned(infos.size()),
                                [&infos](unsigned idx) -> const Skeleton2D* {
                                   return infos[idx].p2d_ptr.get();
                                });

      // Add `prop_fp` to labels
      Expects(pose_labels.size() == infos.size());
      const auto& dcam   = scene_desc->dcam(sensor_no);
      const auto weights = Tracks::Params{}.false_positive_weight_vector;
      vector<string> base_labels(infos.size());
      for(auto i = 0u; i < infos.size(); ++i) {
         const auto& info = infos[i];
         const auto& p2d  = *info.p2d_ptr;

         const auto scores = calc_prob_p2d_fp_scores(
             loc_params, p2d, &info.hist, bounds, hist_sz, dcam);
         const auto prob_fp    = calc_prob_p2d_false_positive(scores, weights);
         pose_labels[i].second = format("id = {}-{}", sensor_no, i);
         base_labels[i]        = trim_copy(format(R"V0G0N(
id    = {}-{}
score = {:4.3f}, {:4.3f}, {:4.3f}
p(fp) = {:4.3f}
heigt = {:3.2f}
X     = ({:2.1f}, {:2.1f})
)V0G0N",
                                           sensor_no,
                                           i,
                                           scores.p2d_score,
                                           scores.p2d_cloud_in_bounds_height,
                                           scores.p2d_shape,
                                           prob_fp,
                                           p2d.best_3d_result().height(),
                                           p2d.best_3d_result().X().x,
                                           p2d.best_3d_result().X().y));
      }
      timings.render_pose_skeleton_s += tock(now);

      if(base_labels.size() > 0) { // Put `base_labels` into `pose_labels`
         Point2 wh = {0, 0};
         for(const auto& lbl : base_labels) {
            const auto lbl_wh = render_tiny_dimensions(lbl);
            wh.x              = std::max(lbl_wh.x, wh.x);
            wh.y              = std::max(lbl_wh.y, wh.y);
         }
         const int base_margin  = 10; // pixels at bottom
         const int base_padding = 24; // between reports
         const int base_h       = int(argb.height) - base_margin - wh.y;

         pose_labels.reserve(pose_labels.size() + base_labels.size());
         for(size_t i = 0; i < base_labels.size(); ++i) {
            const auto pos
                = Point2(base_margin + int(i) * (base_padding + wh.x), base_h);
            pose_labels.emplace_back(pos, std::move(base_labels[i]));
         }
      }
   }

   if(tracks_ptr != nullptr
      && (params.render_tracks_on_raw_video
          || params.render_ground_truth_only)) {
      const auto now = tick();

      if(false) {
         TRACE(format("render-tracks = {}, gt-only = {}, gt = {}",
                      str(params.render_tracks_on_raw_video),
                      str(params.render_ground_truth_only),
                      str(params.render_ground_truth)));
      }

      decltype(gt_tracks_ptr) ptr = nullptr;
      if(!params.render_ground_truth_only && params.render_ground_truth)
         ptr = gt_tracks_ptr;

      const bool gt_style = (ptr != nullptr) && params.render_ground_truth_only;

      render_tracks_on_raw_video(argb,
                                 *scene_desc,
                                 sensor_no,
                                 frame_no,
                                 *tracks_ptr,
                                 ptr,
                                 loc_dat,
                                 gt_style);
      timings.render_tracks_on_raw_video_s += tock(now);
   }

   // Timers
   return {pose_labels, timings};
}

// ------------------------------------------------------------ make debug movie
//
void make_debug_movie(
    const SceneDescription* scene_desc,
    const DebugMovieParams& params,
    const LocalizationData::Params& loc_params,
    const int start_frame,
    const int n_frames,
    std::function<const LocalizationData*(int frame_no)> get_loc,
    std::function<const Tracks*(int frame_no)> get_tracks,
    std::function<const tracks::ComputationData*(int frame_no)> get_comp_data,
    const FowlkesResult* fowlkes_result_ptr,        // could be nullptr
    const pipeline::TestOutput* gt_compare_ret_ptr, // could be nullptr
    const string_view working_dir,
    const string_view out_movie_name) noexcept
{
   TRACE(format("generating debug movie"));

   Expects(scene_desc);
   Expects(start_frame >= 0);

   const int n_cams    = scene_desc->n_cameras();
   const int end_frame = start_frame + n_frames - 1; // includsive
   if(n_frames <= 0) {
      LOG_ERR(format("no frames in debug movie! Start = {}, end = {}",
                     start_frame,
                     end_frame));
      return;
   }

   const auto loc_dat0 = get_loc(start_frame);
   Expects(loc_dat0);
   const real hist_sz        = loc_dat0->hist_sz;
   const AABB bounds         = loc_dat0->bounds;
   const Vector2 top_left    = bounds.top_left();
   const real scene_fps      = scene_desc->scene_fps;
   const real frame_duration = scene_desc->frame_duration();
   const auto floor_grids    = scene_desc->all_floor_grids();

   const vector<Track>* tracks_ptr    = nullptr;
   const vector<Track>* gt_tracks_ptr = nullptr;
   if(fowlkes_result_ptr != nullptr) tracks_ptr = &fowlkes_result_ptr->tracks;
   if(params.render_ground_truth_only && gt_compare_ret_ptr != nullptr)
      tracks_ptr = &gt_compare_ret_ptr->ground_truth;
   else if(gt_compare_ret_ptr != nullptr)
      gt_tracks_ptr = &gt_compare_ret_ptr->ground_truth;

   if(params.render_tracks_on_raw_video and !fowlkes_result_ptr) {
      if(!get_tracks) {
         LOG_ERR(format("attempt to render tracks on a video, but no result "
                        "tracks were given!"));
      } else {
         try {
            FATAL(format("this no long works. The complete tracks-ret from the "
                         "moviereuslts task needs to be made available"));
            // fowlkes_ret        = make_fowlkes_result(start_frame,
            //                                   n_frames,
            //                                   get_tracks,
            //                                   top_left,
            //                                   hist_sz,
            //                                   frame_duration);
            // fowlkes_result_ptr = &fowlkes_ret;
         } catch(std::exception& e) {
            LOG_ERR(format("failed to initialize 'fowlkes-result', which means "
                           "that tracks will not be rendered: {}",
                           e.what()));
         }
      }
   }

   // Seek the first frame for output
   const auto frame0 = scene_desc->get_images_for_frame(start_frame);
   if(!frame0.frame_loaded) {
      LOG_ERR(format("failed to seek frame {}, aborting debug movie creation",
                     start_frame));
      return;
   }

   int out_frame_no = 1; // ffmpeg wants '1' as the first frame

   // Output image dimensions
   const int w              = params.w;
   const int h              = params.h;
   const int margin         = 0;
   const int padding        = 10;
   const int sensor_im_w    = frame0.sensor_images[0].cols;
   const int sensor_im_h    = frame0.sensor_images[0].rows;
   constexpr int n_hist_ims = 5;
   const int hist_w         = int(loc_dat0->loc_hist.width);
   const int hist_h         = int(loc_dat0->loc_hist.height);

   const MovieLayout ml = make_movie_layout(w,
                                            h,
                                            margin,
                                            padding,
                                            n_cams,
                                            sensor_im_w,
                                            sensor_im_h,
                                            n_hist_ims,
                                            hist_w,
                                            hist_h);

   if(!ml.is_init) {
      LOG_ERR(format("failed to initialize MovieLayout"));
      cout << str(ml) << endl << endl;
      ml.debug_image().save(
          format("{}/zzz-debug-movie-layout.png", working_dir));
   }
   Expects(ml.is_init);

   auto encoder = StreamingFFMpegEncoder::create(
       out_movie_name, w, h, 1.0 / frame_duration);

   MovieUtils mutils;
   mutils.hist_w      = unsigned(hist_w);
   mutils.hist_h      = unsigned(hist_h);
   mutils.hist_bounds = bounds;
   mutils.hist_sz     = hist_sz;
   mutils.do_rotate   = ml.rotate_hists;
   mutils.p3d_radius  = params.person_radius;

   ARGBImage sz_bg_image;
   if(scene_desc->has_background_image()) {
      auto bg_big = argb_to_cv(scene_desc->background_image);
      cv::Mat bg_sz;
      cv::resize(bg_big, bg_sz, cv::Size(hist_w, hist_h));
      sz_bg_image = cv_to_argb(bg_sz);
   }

   // Process a single frame, to output the movie data
   auto process_frame = [&](int frame_no) {
      const auto frame_now = tick();

      if(false)
         TRACE(format("generating debug movie, frame #{:4d}/{:4d}",
                      frame_no,
                      end_frame));

      // This is the frame
      cv::Mat out = cv::Mat{h, w, CV_8UC3, cv::Scalar(255, 255, 255)};

      // Seek the frame
      const auto frame_ims = scene_desc->get_images_for_frame(frame_no);
      if(!frame_ims.frame_loaded) {
         FATAL(format("failed to load frame {}, aborting", frame_no));
      }

      // Get the location data for this frame
      const auto loc_dat = get_loc(frame_no);
      Expects(loc_dat != nullptr);

      {
         RenderSensorTimings timings;
         real cv_resize_s = 0.0;

         const auto now0 = tick();
         for(auto i = 0; i < n_cams; ++i) {
            const auto sensor_no = scene_desc->sensor_lookup(i, 0);
            auto argb = cv_to_argb(frame_ims.sensor_images[size_t(sensor_no)]);

            const auto [pose_labels, timings_i]
                = render_sensor(scene_desc,
                                params,
                                loc_params,
                                loc_dat,
                                tracks_ptr,
                                gt_tracks_ptr,
                                frame_no,
                                sensor_no,
                                floor_grids.at(size_t(sensor_no)),
                                argb);
            timings += timings_i;

            { // resize and place
               const auto now = tick();
               cv::Mat im     = argb_to_cv(argb);

               const auto& bounds = ml.movie_bounds[size_t(i)];
               cv::Mat im_small
                   = movie::resize_and_render_labels(im,
                                                     pose_labels,
                                                     unsigned(bounds.width()),
                                                     unsigned(bounds.height()));

               im_small.copyTo(out(to_cv_rect(bounds)));
               cv_resize_s += tock(now);
            }

            if(false) { // Export p2d skeletons
               const auto& infos = loc_dat->p2ds[size_t(sensor_no)];
               for(auto i = 0u; i < infos.size(); ++i) {
                  const Skeleton2D& p2d = *infos[i].p2d_ptr;
                  const auto fname      = format(
                      "{}/frame-{:04d}_sensor-{:02d}_skeleton-{:02d}.json",
                      working_dir,
                      frame_no,
                      sensor_no,
                      i);
                  file_put_contents(fname, p2d.to_json_str());
                  TRACE(format("skeleton saved to {}", fname));
               }
               cv_to_argb(frame_ims.sensor_images[size_t(sensor_no)])
                   .save(format("{}/frame-{:04d}_sensor-{:02d}.png",
                                working_dir,
                                frame_no,
                                sensor_no));
            }
         }

         if(params.print_timings) {
            const auto tot_frame_time_s = tock(now0);
            cout << format(R"V0G0N(TIMINGS (milliseconds):
   blur:             {: 4d}
   floor-grid:       {: 4d}
   p2d-sparse-hists: {: 4d}
   p2d-height-line:  {: 4d}
   p2d-cylinders:    {: 4d}
   pose-skeletons:   {: 4d}
   track-result:     {: 4d}
   resize-and-blit:  {: 4d}
   ----------------------
   total-pds:        {: 4d}
   total-time:       {: 4d}
)V0G0N",
                           int(1000.0 * timings.blur_faces_s),
                           int(1000.0 * timings.render_grid_s),
                           int(1000.0 * timings.render_p2d_hists_s),
                           int(1000.0 * timings.render_heights_line_s),
                           int(1000.0 * timings.render_p2d_cylinders_s),
                           int(1000.0 * timings.render_pose_skeleton_s),
                           int(1000.0 * timings.render_tracks_on_raw_video_s),
                           int(1000.0 * cv_resize_s),
                           timings.total_p2ds,
                           int(1000.0 * tot_frame_time_s));
         }
      }

      auto finish_image = [&](const string_view label,
                              const int hist_im_n,
                              const ARGBImage& argb,
                              std::function<void(cv::Mat&)> post_process_f) {
         cv::Mat m = argb_to_cv(argb);
         cv::Mat sized_im;
         if(ml.rotate_hists) cv::rotate(m, m, cv::ROTATE_90_CLOCKWISE);
         Expects(unsigned(hist_im_n) < ml.hist_bounds.size());
         const auto& bounds = ml.hist_bounds[size_t(hist_im_n)];
         cv::resize(m, sized_im, cv::Size(bounds.width(), bounds.height()));

         if(!label.empty()) {
            const Point2 xy = render_tiny_dimensions(label);
            const auto dx   = (sized_im.cols - xy.x) / 2;
            const auto dy   = sized_im.rows - xy.y - 2;
            render_string_cv(
                sized_im, label, Point2(dx, dy), k_yellow, k_black);
         }

         if(post_process_f) post_process_f(sized_im);

         sized_im.copyTo(out(to_cv_rect(bounds)));
      };

      auto make_background = [&]() {
         ARGBImage argb = grey16_im_to_argb(loc_dat->loc_hist);
         if(sz_bg_image.width == argb.width)
            blend_images(argb, sz_bg_image, 0.75);
         return argb;
      };
      const auto bg_image = make_background();

      { // Localization image
         const auto now = tick();
         ARGBImage argb = grey16_im_to_argb(loc_dat->loc_hist);

         auto f = [&](cv::Mat& im) {
            if(params.grid_on_localization) mutils.render_grid(im, bounds);
         };

         finish_image("localization", 0, argb, f);
      }

      { // Detections
         const auto now = tick();
         ARGBImage argb = grey16_im_to_argb(loc_dat->loc_hist);
         auto f         = [&](cv::Mat& im) {
            mutils.render_grid(im, bounds);
            mutils.render_p2ds(im, *loc_dat, true, false);
         };
         finish_image("detections", 1, argb, f);
      }

      { // Graph Image
         const auto now      = tick();
         const auto comp_dat = get_comp_data(frame_no);
         ARGBImage argb      = mutils.make_empty_hist();
         auto f              = [&](cv::Mat& im) {
            mutils.render_grid(im, bounds);
            mutils.render_comp_graph(im, frame_no, *comp_dat, true);
         };
         finish_image("XYTs", 2, argb, f);
      }

      { // Tracks Image
         const auto now       = tick();
         const Tracks* tracks = get_tracks ? get_tracks(frame_no) : nullptr;
         if(tracks == nullptr) {
            finish_image("tracks", 3, mutils.make_empty_hist(), nullptr);
         } else {
            auto f = [&](cv::Mat& im) {
               mutils.render_grid(im, bounds);
               mutils.render_track(im, frame_no, tracks->seqs);
            };
            finish_image("tracks", 3, bg_image, f);
         }
      }

      { // Fowlkes Image
         const auto now = tick();
         if(fowlkes_result_ptr == nullptr && gt_compare_ret_ptr == nullptr) {
            finish_image("out", 4, mutils.make_empty_hist(), nullptr);
         } else {
            auto f = [&](cv::Mat& im) {
               mutils.render_grid(im, bounds);
               if(gt_compare_ret_ptr != nullptr
                  && !params.render_ground_truth_only) {
                  mutils.render_track_match(
                      im, unsigned(frame_no), *gt_compare_ret_ptr);
               } else if(tracks_ptr != nullptr) {
                  mutils.render_track(im, frame_no, *tracks_ptr);
               } else {
                  FATAL("logic error");
               }
            };
            finish_image("final", 4, bg_image, f);
         }
      }

      // Draw the frame-number
      mutils.render_frame_no(out, unsigned(frame_no));
      encoder.push_frame(out);
   };

   // ... // ... // For each of the frames

   for(auto frame_no = start_frame; frame_no <= end_frame; ++frame_no) {
      const auto print_timing = (frame_no % 20 == 0);
      const auto now          = tick();
      if(print_timing)
         cout << format("debug-movie frame {:4d}/{:4d}", frame_no, end_frame)
              << std::flush;
      process_frame(frame_no);
      if(print_timing)
         cout << format(" -- {:5.3f}s per frame", tock(now)) << endl;
   }

   if(encoder.close() != 0) { FATAL(format("ffmpeg failed")); }
}

// ------------------------------------------------------------ make debug movie
//
void make_debug_movie(const PipelineOutput& pipeline_output,
                      const FowlkesResult& fowlkes_result,
                      const pipeline::TestOutput* gt_compare_ptr,
                      pipeline::MovieResults* movie,
                      const DebugMovieParams& params) noexcept
{
   const auto p                       = movie->params();
   const SceneDescription* scene_desc = p.scene_desc.get();
   const auto movie_ret  = movie->movie_task.get_result_synchronized();
   const auto tracks_ret = movie->tracks_task.get_result_synchronized();
   const int start_frame = int(movie_ret->start_frame());
   const int end_frame   = movie_ret->end_frame();
   const int n_frames    = int(movie_ret->n_frames());

   //
   movie->set_feedback(false);

   int calc_idx                                         = -1;
   pipeline::tracks::Result::CompDataEnvelope comp_data = {};

   auto get_comp_data
       = [&](int frame_no) -> const perceive::tracks::ComputationData* {
      const auto idx = tracks_ret->frame_no_to_track_object_idx(frame_no);
      if(idx != calc_idx) {
         calc_idx  = idx;
         comp_data = tracks_ret->calc_comp_data(calc_idx);
      }
      return comp_data.data_ptr.get();
   };

   LocalizationData::Params loc_params;
   try {
      loc_params = pipeline_output.localization_params();
   } catch(std::exception& e) {
      LOG_ERR(format("failed to read localization params"));
   }

   const auto out_fname = format("{}/debug-video.mp4", p.config.outdir);

   make_debug_movie(
       scene_desc,
       params,
       loc_params,
       start_frame,
       n_frames,
       [&](int frame_no) {
          return movie_ret->localization_result(unsigned(frame_no)).get();
       },
       [&](int frame_no) { return tracks_ret->tracks(frame_no); },
       get_comp_data,
       &fowlkes_result,
       gt_compare_ptr,
       p.config.outdir,
       out_fname);

   INFO(format("debug-movie saved to '{}'", out_fname));

   return;
}

} // namespace perceive::movie
