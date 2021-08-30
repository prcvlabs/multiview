
#include "movie-utils.hpp"

#include "perceive/cost-functions/tracks/labeled-track-point.hpp"
#include "perceive/cost-functions/tracks/ops.hpp"
#include "perceive/geometry/human-heights.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/io/lazy-s3.hpp"

#define This MovieUtils

namespace perceive::movie
{
// ------------------------------------------------------------------ blur-faces
//
template<typename T>
void blur_faces_T(
    T& im,
    const int frame_no,
    const bool draw_pose,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept
{
   constexpr bool is_cv_version = std::is_same<T, cv::Mat>::value;

   auto calc_width = [&im]() -> int {
      if constexpr(is_cv_version)
         return im.cols;
      else
         return int(im.width);
   };

   auto calc_height = [&im]() -> int {
      if constexpr(is_cv_version)
         return im.cols;
      else
         return int(im.width);
   };

   const AABBi im_bounds(0, 0, calc_width(), calc_height());
   const auto now = tick();

   for(auto i = 0u; i < n_poses; ++i) {
      const auto pose = get_pose_ptr(i);
      AABBi head_roi  = intersection(im_bounds, pose->head_roi());
      const auto X    = pose->head();
      if(head_roi.area() == 0) {
         if(false) {
            WARN(format("frame #{:4d}, failed to find head on pose... skipping",
                        frame_no));
         }
      } else {
         if(head_roi.width() < 15) head_roi.grow(15 - head_roi.width());
         if(head_roi.height() < 15) head_roi.grow(15 - head_roi.height());

         const AABBi roi       = intersection(im_bounds, head_roi);
         const cv::Rect cv_roi = to_cv_rect(roi);
         if(cv_roi.width <= 0 || cv_roi.height <= 0) continue;

         cv::Mat part;
         if constexpr(is_cv_version) {
            part = im(cv_roi);
         } else {
            part = crop_to_cv(im, roi);
         }

         cv::Mat blurred = part.clone();
         cv::blur(part, blurred, cv::Size(cv_roi.width, cv_roi.height));

         cv::Mat mask = cv::Mat(part.rows, part.cols, CV_8U);
         mask.setTo(0);
         cv::RotatedRect rr{
             cv::Point2f(0.0f, 0.0f),
             cv::Point2f(float(cv_roi.width), 0.0f),
             cv::Point2f(float(cv_roi.width), float(cv_roi.height))};
         cv::ellipse(mask, rr, 255, -1);

         if constexpr(is_cv_version) {
            blurred.copyTo(im(cv_roi), mask);
         } else {
            blurred.copyTo(part, mask);
            blit(part, im, roi);
         }
      }

      if(draw_pose) { render_pose(im, *pose); }
   }
}

void blur_faces(
    cv::Mat& im,
    const int frame_no,
    const bool draw_pose,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept
{
   blur_faces_T(im, frame_no, draw_pose, n_poses, get_pose_ptr);
}

void blur_faces(
    ARGBImage& argb,
    const int frame_no,
    const bool draw_pose,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept
{
   blur_faces_T(argb, frame_no, draw_pose, n_poses, get_pose_ptr);
}

void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const int sensor_no,
                const vector<Pose3D>& p3ds) noexcept
{
   vector<const Skeleton2D*> poses;
   poses.reserve(100);

   for(const auto& p3d : p3ds) {
      for(const auto& sp : p3d.poses()) {
         if(sp.sensor_no == sensor_no) { poses.push_back(sp.pose.get()); }
      }

      blur_faces(im, frame_no, draw_pose, poses);
   }
}

void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const vector<const Skeleton2D*>& poses) noexcept
{
   blur_faces(
       im, frame_no, draw_pose, unsigned(poses.size()), [&](unsigned idx) {
          return poses[idx];
       });
}

void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const vector<shared_ptr<const Skeleton2D>>& poses) noexcept
{
   blur_faces(
       im, frame_no, draw_pose, unsigned(poses.size()), [&](unsigned idx) {
          return poses[idx].get();
       });
}

// ----------------------------------------------------------------- render-pose
//
vector<std::pair<Point2, string>> render_poses(
    ARGBImage& argb,
    const int sensor_no,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept
{
   vector<std::pair<Point2, string>> op_labels;

   auto label_pos = [&](const auto& pose) {
      const auto aabb = pose.aabb();
      return to_pt2(Vector2(0.5 * (aabb.right + aabb.left), aabb.top - 7));
   };

   int counter = 0;
   for(auto i = 0u; i < n_poses; ++i) {
      const Skeleton2D* pose_ptr = get_pose_ptr(i);
      if(sensor_no != pose_ptr->sensor_no()) continue;
      render_pose(argb, *pose_ptr);
      string label = format("{},{}", sensor_no, counter++);
      op_labels.emplace_back(label_pos(*pose_ptr), std::move(label));
   }

   return op_labels;
}

vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<Pose3D>& p3ds) noexcept
{
   vector<std::pair<Point2, string>> op_labels;

   auto label_pos = [&](const auto& pose) {
      const auto aabb = pose->aabb();
      return to_pt2(Vector2(0.5 * (aabb.right + aabb.left), aabb.top - 7));
   };

   int counter = 0;
   for(const auto& p3d : p3ds) {
      for(const auto& pose : p3d.poses()) {
         if(pose.sensor_no == sensor_no) {
            render_pose(argb, *pose.pose);
            string label = format("{},{}", sensor_no, counter++);
            op_labels.emplace_back(label_pos(pose.pose), std::move(label));
         }
      }
   }

   return op_labels;
}

vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<shared_ptr<const Skeleton2D>>& p2ds) noexcept
{
   return render_poses(argb,
                       sensor_no,
                       unsigned(p2ds.size()),
                       [&](unsigned idx) { return p2ds[idx].get(); });
}

vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<Skeleton2DInfo>& p2d_infos) noexcept
{
   return render_poses(argb,
                       sensor_no,
                       unsigned(p2d_infos.size()),
                       [&](unsigned idx) -> const Skeleton2D* {
                          return p2d_infos[idx].p2d_ptr.get();
                       });
}

// ---------------------------------------------------- resize-and-render-labels
//
cv::Mat
resize_and_render_labels(const cv::Mat& im,
                         const vector<std::pair<Point2, string>>& labels,
                         unsigned new_w,
                         unsigned new_h) noexcept
{
   cv::Mat im_small;
   cv::resize(im, im_small, cv::Size(int(new_w), int(new_h)));

   for(const auto& lbls : labels) {
      const auto& X     = lbls.first;
      const auto& label = lbls.second;

      const auto Y = Vector2(X.x * real(new_w) / real(im.cols),
                             X.y * real(new_h) / real(im.rows));

      render_string_f(label, k_yellow, k_black, [&](int x, int y, uint32_t k) {
         set(im_small, int(x + Y.x), int(y + Y.y), rgb_to_vec3b(k));
      });

      if(false) {
         INFO(format("{}, {} => {} => {}",
                     lbls.second,
                     str(lbls.first),
                     str(X),
                     str(Y)));
      }
   }

   return im_small;
}

// ------------------------------------------------------------- make-background
//
void blend_images(ARGBImage& A, const ARGBImage& B, const float alpha) noexcept
{
   Expects(A.width == B.width);
   Expects(A.height == B.height);
   const int h = int(A.height);
   const int w = int(A.width);

   for(auto y = 0; y < h; ++y) {
      auto row0 = A.row_ptr(unsigned(y));
      auto row1 = B.row_ptr(unsigned(y));
      for(auto x = 0; x < w; ++x) { row0[x] = blend(row1[x], row0[x], alpha); }
   }
}

// ------------------------------------------------------------- make-empty-hist
//
ARGBImage This::make_empty_hist() const noexcept
{
   ARGBImage im;
   im.resize(hist_w, hist_h);
   im.fill(k_light_gray);
   auto f = [&](int x, int y) {
      if(im.in_bounds(x, y)) im(x, y) = k_black;
   };
   bresenham(Point2(0, 0), Point2(int(hist_w), 0), f);
   bresenham(Point2(int(hist_w), 0), Point2(int(hist_w), int(hist_h)), f);
   bresenham(Point2(int(hist_w), int(hist_h)), Point2(0, int(hist_h)), f);
   bresenham(Point2(0, int(hist_h)), Point2(0, 0), f);
   return im;
}

// -------------------------------------------------------------- hist-X stretch
//
real This::hist_X_to_image_strech_w(const cv::Mat& im) const noexcept
{
   return real(im.cols) / real(hist_w);
}

real This::hist_X_to_image_strech_h(const cv::Mat& im) const noexcept
{
   return real(im.rows) / real(hist_h);
}

// ------------------------------------------------------------- hist-X-to-image
//
Vector2 This::hist_X_to_image(const cv::Mat& im,
                              const Vector2& X) const noexcept
{
   const int im_w = im.cols, im_h = im.rows;
   Vector2 xy = X;
   if(do_rotate) {
      // Scale
      const real xx = X.x * real(im_h) / real(hist_w);
      const real yy = X.y * real(im_w) / real(hist_h);

      // Now perform the rotate
      xy.x = real(im_w) - yy;
      xy.y = xx;
   } else {
      // Scale
      xy.x = X.x * real(im_w) / real(hist_w);
      xy.y = X.y * real(im_h) / real(hist_h);
   }
   return xy;
}

// ---------------------------------------------------------------- label tracks
//
void This::label_tracks(cv::Mat& im,
                        const int frame_no,
                        const vector<Track>& tracks) const noexcept
{
   const int im_w = im.cols, im_h = im.rows;

   auto do_label = [&](const int id, const Vector2& X) {
      Vector2 xy = hist_X_to_image(im, X);

      // Then render 'id' as text on 'im'
      const string label = format("{}", id);
      const auto dim     = render_tiny_dimensions(label);
      const Point2 pos   = Point2(int(std::round(xy.x - 0.5 * dim.x)),
                                int(std::round(xy.y - 0.5 * dim.y)));
      render_string_cv(im, label, pos, k_black);
   };

   for(const auto& tt : tracks)
      for(const auto& tp : tt.path)
         if(tp.t == frame_no) do_label(tt.id, to_vec2(tp.xy()));
}

// ------------------------------------------------------------ world X to image
//
Vector2 This::world_X_to_image(const cv::Mat& im,
                               const Vector3& X) const noexcept
{
   const Vector2 x = FloorHistogram::project_to_histogram_f(
       X, hist_bounds.top_left(), hist_sz);
   return hist_X_to_image(im, x);
}

// ----------------------------------------------------------------- render grid
//
void This::render_grid(cv::Mat& im, const AABB& aabb) const noexcept
{
   check_image_(im);

   auto draw_line = [&](const Vector2& U,
                        const Vector2& V,
                        const uint32_t k,
                        const bool do_stipple) {
      const auto cv_k = rgb_to_vec3b(k);
      const auto u    = world_X_to_image(im, U);
      const auto v    = world_X_to_image(im, V);
      int counter     = 0;
      bresenham(u, v, [&](int x, int y) {
         const bool do_draw = !do_stipple or (((counter++) / 4) % 2);
         if(do_draw) set(im, x, y, cv_k);
         counter++;
      });

      if(counter > 1000000) {
         TRACE(format("GRID, line: ({} -> {}), stipple = {}, hist-sz = {}, tl "
                      "= {}, ({} -> {})",
                      str(U),
                      str(V),
                      str(do_stipple),
                      hist_sz,
                      str(hist_bounds.top_left()),
                      str(u),
                      str(v)));
      }
   };

   const int y0 = int(std::floor(aabb.top));
   const int y1 = int(std::ceil(aabb.bottom));
   const int x0 = int(std::floor(aabb.left));
   const int x1 = int(std::floor(aabb.right));
   for(int y = y0; y <= y1; ++y)
      draw_line(Vector2(x0, y), Vector2(x1, y), k_orange, true);
   for(int x = x0; x <= x1; ++x)
      draw_line(Vector2(x, y0), Vector2(x, y1), k_orange, true);

   draw_line(Vector2(0.0, 0.0), Vector2(0.5, 0.0), k_red, false);
   draw_line(Vector2(0.0, 0.0), Vector2(0.0, 0.5), k_green, false);
}

// ------------------------------------------------------------------ render p3d
//
void This::render_p3d(cv::Mat& im,
                      const Vector2& X, // hist position
                      const uint32_t kolour,
                      const string_view label,
                      const float view_direction,
                      const bool fill,
                      const uint32_t gaze_colour) const noexcept
{
   const auto now = tick();

   check_image_(im);

   const Vector2 xy = hist_X_to_image(im, X);                   // center
   const Point2 C   = Point2(int(xy.x + 0.5), int(xy.y + 0.5)); // rounded

   const float radius_r
       = float(hist_X_to_image_strech_w(im) * p3d_radius / hist_sz);

   auto test_time = [&]() {
      if(tock(now) <= 2.0) return; // 2s is okay
   };

   { // draw the circle
      const int radius = int(std::round(radius_r));
      if(fill) {
         // INFO(format(" im [{}x{}], C={}, kolour=0x{:08x}, radius={}",
         //             im.cols,
         //             im.rows,
         //             str(C),
         //             kolour,
         //             radius));
         fill_circle(im, C, kolour, radius, 0.5);
      } else {
         cv::circle(im, cv::Point(C.x, C.y), radius, rgb_to_vec3b(kolour), 2);
      }
   }

   if(std::isfinite(view_direction)) {
      const float dx       = std::cos(view_direction) * radius_r;
      const float dy       = std::sin(view_direction) * radius_r;
      const cv::Vec3b k    = rgb_to_vec3b(gaze_colour);
      const AABB im_bounds = AABB(0, 0, im.cols, im.rows);

      plot_line_AA(to_vec2(C),
                   to_vec2(C) + to_vec2(Vector2f(dx, dy)),
                   im_bounds,
                   [&](int x, int y, float alpha) {
                      if(in_bounds(im, x, y)) {
                         cv::Vec3b& p = im.at<cv::Vec3b>(y, x);
                         p            = k;
                      }
                   });
   }

   if(!label.empty()) { // draw the label
      const auto dim   = render_tiny_dimensions(label);
      const Point2 pos = to_pt2(
          Vector2(std::round(xy.x) - dim.x / 3, std::round(xy.y) - dim.y / 3));
      render_string_cv(im, label, pos, k_yellow, k_black);
   }
}

// ------------------------------------------------------------------- render tp
//
void This::render_tp(cv::Mat& im,
                     const int id,
                     const TrackPoint& tp) const noexcept
{
   render_p3d(im,
              to_vec2(tp.xy()),
              colour_set_4(unsigned(id)),
              format("{},{}", id, pose_to_char(tp.pose)),
              tp.gaze_direction,
              true);
}

// ---------------------------------------------------------------- render track
//
void This::render_track(cv::Mat& im,
                        const int frame_no,
                        const vector<tracks::NodeSequence>& seqs) const noexcept
{
   auto render_node
       = [this](cv::Mat& im, const int id, const tracks::Node& node) {
            render_p3d(im,
                       to_vec2(node.xy()),
                       colour_set_4(unsigned(id)),
                       format("{}", id),
                       node.gaze(),
                       true,
                       k_white);
         };

   for(const auto& seq : seqs)
      for(const auto& node : seq.nodes())
         if(frame_no == node.t()) render_node(im, seq.id(), node);
}

// ---------------------------------------------------------------- render track
//
void This::render_track(cv::Mat& im,
                        const int frame_no,
                        const vector<Track>& tracks) const noexcept
{
   Expects(tracks.size() < 1000000);
   for(const auto& tt : tracks) {
      Expects(tt.path.size() < 1000000);
      for(const auto& tp : tt.path)
         if(frame_no == tp.t) render_tp(im, tt.id, tp);
   }
}

// ---------------------------------------------------------- render track match
//
void This::render_track_match(
    cv::Mat& im,
    const unsigned frame_no,
    const pipeline::TestOutput& compare) const noexcept

{
   auto draw_false_positive = [&](const LabeledTrackPoint& ltp) {
      render_p3d(im,
                 to_vec2(ltp.tp.xy()),
                 colour_set_4(unsigned(ltp.track_id)),
                 format("{}", ltp.track_id),
                 ltp.tp.gaze_direction,
                 true,
                 k_white);

      const real radius
          = std::round(hist_X_to_image_strech_w(im) * p3d_radius / hist_sz);

      const Vector2 C   = hist_X_to_image(im, to_vec2(ltp.tp.xy()));
      const cv::Vec3b k = rgb_to_vec3b(k_red);
      const auto dxy
          = Vector2(cos(to_radians(-45.0)), sin(to_radians(-45.0))) * radius;
      auto f = [&](int x, int y, float alpha) {
         if(in_bounds(im, x, y)) {
            cv::Vec3b& p = im.at<cv::Vec3b>(y, x);
            p            = k;
         }
      };

      plot_line_AA(C - dxy, C + dxy, f);
      plot_line_AA(C + dxy, C - dxy, f);

      // const auto kolour = colour_set_4(ltp.track_id);
      cv::circle(im, cv::Point(int(C.x), int(C.y)), int(radius), k, 2);
   };

   auto draw_true_positive_calc
       = [&](const int track_id, const LabeledTrackPoint& ltp) {
            render_p3d(im,
                       to_vec2(ltp.tp.xy()),
                       colour_set_4(unsigned(ltp.track_id)),
                       format("{}", ltp.track_id),
                       ltp.tp.gaze_direction,
                       true,
                       k_white);
         };

   auto draw_gt = [&](const auto& gt) {
      const LabeledTrackPoint& ltp = gt.ltp;
      const uint32_t gaze_k        = (gt.is_theta_error) ? k_red : k_black;
      const uint32_t k             = colour_set_4(unsigned(ltp.track_id));

      render_p3d(im,
                 to_vec2(ltp.tp.xy()),
                 (gt.is_track_break ? invert_colour(k) : k),
                 format("{}", ltp.track_id),
                 ltp.tp.gaze_direction,
                 false,
                 gaze_k);
   };

   auto get_frame_index = [](auto& frames, int frame_no) {
      auto ii = std::find_if(cbegin(frames), cend(frames), [frame_no](auto& o) {
         return o.calc_t == frame_no;
      });
      return (ii == cend(frames)) ? -1 : std::distance(cbegin(frames), ii);
   };

   // Load the frame info...
   const int frame_index = int(get_frame_index(compare.frames, int(frame_no)));
   if(unsigned(frame_index) >= compare.frames.size())
      return; // index out of range =(

   const auto& f_info = compare.frames[size_t(frame_index)];

   // Draw the false positives
   Expects(f_info.ltp_calc.size() == f_info.calc_matches.size());
   for(auto i = 0u; i < f_info.ltp_calc.size(); ++i)
      if(f_info.calc_matches[i] == -1) draw_false_positive(f_info.ltp_calc[i]);

   for(const auto& gt : f_info.ltp_gt) {
      const bool is_true_positive = (gt.calc_index >= 0);
      if(is_true_positive)
         draw_true_positive_calc(gt.ltp.track_id,
                                 f_info.ltp_calc[size_t(gt.calc_index)]);
      draw_gt(gt);
   }
}

// ----------------------------------------------------------------- render p3ds
//
void This::render_p2ds(cv::Mat& im,
                       const LocalizationData& ldat,
                       const bool render_labels,
                       const bool render_info_ids,
                       const bool fill_p2ds,
                       const bool draw_gaze) const noexcept
{
   check_image_(im);

   const int im_w = im.cols, im_h = im.rows;
   const int radius
       = int(std::round(hist_X_to_image_strech_w(im) * 0.6 / hist_sz));

   auto render_one_info = [&](const Skeleton2DInfo& info, const int label) {
      const uint32_t kolour = colour_set_4(unsigned(info.id));
      const auto& p2d       = *info.p2d_ptr;

      // label_tracks(im, frame_no, tracklet->tracks.tracks);
      const Vector3f C = p2d.best_3d_result().Xs_centre();
      if(!C.is_finite()) return; // nothing to do

      const Vector2 X = ldat.project_to_hist(to_vec3(C));
      string label_s  = ""s;
      if(render_labels && render_info_ids) {
         label_s = format("id={} {},{}", info.id, p2d.sensor_no(), label);
      } else if(render_labels) {
         label_s = format("{},{}", p2d.sensor_no(), label);
      } else if(render_info_ids) {
         label_s = format("{}", info.id);
      }

      const float gaze = draw_gaze ? float(p2d.theta()) : fNAN;
      render_p3d(im, X, kolour, label_s, gaze, fill_p2ds);
   };

   for(const auto& sensor_infos : ldat.p2ds) {
      int counter = 0;
      for(const auto& info : sensor_infos) render_one_info(info, counter++);
   }
}

// --------------------------------------------------------------- render merged
//
void This::render_merged(cv::Mat& im, const vector<Pose3D>& p3ds) const noexcept
{
   check_image_(im);

   int counter = 0;
   for(const auto& p3d : p3ds) {
      const auto hist_X = world_X_to_image(im, p3d.C());
      const auto label  = format("{}", counter);
      const auto kolour = colour_set_4(unsigned(counter));

      render_p3d(im, hist_X, kolour, label, float(p3d.gaze_theta()), true);
      ++counter;
   }
}

// ------------------------------------------------------------- render frame no
//
void This::render_frame_no(cv::Mat& im, const unsigned frame_no) const noexcept
{ // Draw the frame-number
   check_image_(im);

   string label    = format("{}", frame_no);
   const Point2 xy = render_tiny_dimensions(label);
   const auto dx   = im.cols - xy.x - 2;
   const auto dy   = im.rows - xy.y - 2;
   render_string_cv(im, label, Point2(dx, dy), k_black, k_white);
}

// ----------------------------------------------------------------- check-image
//
void This::check_image_(cv::Mat& im) const noexcept
{
   if(im.empty()) FATAL("image parameter not set");
   if(im.type() != CV_8UC3) FATAL("image has the wrong type, expected CV_8UC3");
}

// ----------------------------------------------------------- render-trackpoint
//
static void circle_points(const Vector3& C,
                          const Vector3& N,
                          const real r,
                          const unsigned n_divisions,
                          std::function<void(const Vector3&)> f) noexcept
{
   auto q    = Quaternion::between_vectors(Vector3(0, 0, 1), N);
   Vector3 X = q.rotate(Vector3(1, 0, 0));
   Vector3 Y = q.rotate(Vector3(0, 1, 0));

   const auto n = n_divisions;
   real step    = 2.0 * M_PI / real(n);

   for(auto i = 0u; i <= n; ++i) {
      const auto dxy = Vector2(cos(i * step), sin(i * step));
      const auto U   = C + r * (dxy.x * X + dxy.y * Y);
      f(U);
   }
}

void This::render_trackpoint(ARGBImage& im,
                             const DistortedCamera& dcam,
                             const int track_id,
                             const TrackPoint& tp,
                             const bool render_icon) const noexcept
{
   const auto top_left = hist_bounds.top_left();
   auto proj_to_hist   = [&](const Vector3& X) -> Vector2 {
      return FloorHistogram::project_to_histogram(X, top_left, hist_sz);
   };

   auto hist_to_floor = [&](const Vector2& x) -> Vector3 {
      const auto X = Vector2(x.x, x.y) * hist_sz + top_left;
      return Vector3(X.x, X.y, 0.0);
   };

   const Vector3 C0 = hist_to_floor(to_vec2(tp.xy()));
   const auto C     = C0 + Vector3{0.0, 0.0, 0.01};
   const auto Z     = Vector3(0, 0, 1);
   const auto e     = 0.15;
   const real r     = 0.3;
   const real theta = real(tp.gaze_direction);

   const auto max_line_len = 15.0;
   const uint32_t kolour   = colour_set_4(unsigned(track_id));

   const AABB im_bounds = AABB(0, 0, im.width, im.height);

   auto draw_it = [&](int x, int y, float a) {
      if(!im.in_bounds(x, y)) return;
      im(x, y) = blend(im(x, y), kolour, 1.0f - a);
   };

   auto draw_it2 = [&](int x, int y, float a) {
      if(!im.in_bounds(x, y)) return;
      im(x, y) = blend(im(x, y), kolour, 0.0f);
   };

   auto plot_circle = [&](const real r) {
      Vector2 last = Vector2::nan();
      circle_points(C, Z, r, 50, [&](const Vector3& X) {
         const Vector2 x = project_to_distorted(dcam, X);
         if(last.is_finite() and (x - last).norm() < max_line_len)
            plot_line_AA(last, x, draw_it2, false, 5);
         last = x;
      });
   };
   plot_circle(r - 0.00);
   plot_circle(r - 0.01);
   plot_circle(r - 0.02);
   plot_circle(r - 0.03);

   if(std::isfinite(theta)) {
      const auto dxy = Vector3(cos(theta), sin(theta), 0);
      const real phi = acos(r / (r + e));

      const auto W = C + (r + e) * dxy;
      const auto A = C + r * Vector3(cos(theta - phi), sin(theta - phi), 0);
      const auto B = C + r * Vector3(cos(theta + phi), sin(theta + phi), 0);

      const auto a = project_to_distorted(dcam, A);
      const auto w = project_to_distorted(dcam, W);
      const auto b = project_to_distorted(dcam, B);

      plot_line_AA(a, w, im_bounds, draw_it2, false, 7);
      plot_line_AA(w, b, im_bounds, draw_it2, false, 7);
   }

   if(render_icon) render_pose_icon(im, dcam, tp.rounded_xy(), tp.pose);
}

void This::render_gt_trackpoint(ARGBImage& im,
                                const DistortedCamera& dcam,
                                const TrackPoint& tp,
                                const bool render_icon) const noexcept
{
   const auto top_left = hist_bounds.top_left();
   auto proj_to_hist   = [&](const Vector3& X) -> Vector2 {
      return FloorHistogram::project_to_histogram(X, top_left, hist_sz);
   };

   auto hist_to_floor = [&](const Vector2& x) -> Vector3 {
      const auto X = Vector2(x.x, x.y) * hist_sz + top_left;
      return Vector3(X.x, X.y, 0.0);
   };

   const Vector3 C0 = hist_to_floor(to_vec2(tp.xy()));
   const auto C     = C0 + Vector3{0.0, 0.0, 0.01};
   const auto Z     = Vector3(0, 0, 1);
   const auto e     = 0.15;
   const real r     = 0.25;
   const real theta = real(tp.gaze_direction);

   const auto max_line_len = 15.0;
   const uint32_t kolour   = k_gray;

   const AABB im_bounds = AABB(0, 0, im.width, im.height);

   auto draw_it = [&](int x, int y, float a) {
      if(!im.in_bounds(x, y)) return;
      im(x, y) = blend(im(x, y), kolour, 1.0f - a);
   };

   auto draw_it2 = [&](int x, int y, float a) {
      if(!im.in_bounds(x, y)) return;
      im(x, y) = blend(im(x, y), kolour, 0.0f);
   };

   auto plot_circle = [&](const real r) {
      Vector2 last = Vector2::nan();
      circle_points(C, Z, r, 50, [&](const Vector3& X) {
         const Vector2 x = project_to_distorted(dcam, X);
         if(last.is_finite() and (x - last).norm() < max_line_len)
            plot_line_AA(last, x, draw_it2, false, 5);
         last = x;
      });
   };
   plot_circle(r - 0.00);

   if(std::isfinite(theta)) {
      const auto dxy = Vector3(cos(theta), sin(theta), 0);
      const real phi = acos(r / (r + e));

      const auto W = C + (r + e) * dxy;
      const auto A = C + r * Vector3(cos(theta - phi), sin(theta - phi), 0);
      const auto B = C + r * Vector3(cos(theta + phi), sin(theta + phi), 0);

      const auto a = project_to_distorted(dcam, A);
      const auto w = project_to_distorted(dcam, W);
      const auto b = project_to_distorted(dcam, B);

      plot_line_AA(a, w, im_bounds, draw_it2, false, 7);
      plot_line_AA(w, b, im_bounds, draw_it2, false, 7);
   }

   if(render_icon) render_pose_icon(im, dcam, tp.rounded_xy(), tp.pose);
}

void This::render_pose_icon(ARGBImage& im,
                            const DistortedCamera& dcam,
                            const Point2& hist_xy,
                            const ARGBImage& icon) const noexcept
{
   const auto top_left = hist_bounds.top_left();

   auto hist_to_floor = [&](const Point2& x) -> Vector3 {
      const auto X = Vector2(x.x, x.y) * hist_sz + top_left;
      return Vector3(X.x, X.y, 0.0);
   };

   const Vector3 C0 = hist_to_floor(hist_xy);
   const Point2 X0  = to_pt2(project_to_distorted(dcam, C0));
   const Point2 X   = X0 - Point2(icon.width / 2, icon.height / 2);

   // Now blit the `icon` onto `im`
   for(unsigned y = 0; y < icon.height; ++y) {
      const int dst_row = X.y + int(y);
      if(unsigned(dst_row) >= im.height) continue; // out of bounds
      uint32_t* dst       = im.row_ptr(unsigned(dst_row)) + X.x;
      const uint32_t* src = icon.row_ptr(y);

      for(unsigned x = 0; x < icon.width; ++x) {
         if(unsigned(X.x) + x >= im.width) continue; // out of bounds
         dst[x] = blend(src[x], dst[x], 1.0f - (alpha(src[x]) / 255.0f));
      }
   }
}

void This::render_pose_icon(ARGBImage& im,
                            const DistortedCamera& dcam,
                            const Point2& hist_xy,
                            const PoseAnnotation pose) const noexcept
{
   auto calc_icons = []() {
      std::array<ARGBImage, n_pose_annotations()> o;
      auto load_image = [&](const string_view name, const PoseAnnotation pose) {
         const size_t ind = size_t(pose);
         Expects(ind < o.size());
         const auto path = format("s3://perceive-multiview/assets/icons/{}", name);
         try {
            vector<char> buffer;
            lazy_load(path, buffer);
            cv::Mat im = decode_image_to_cv_mat(buffer);
            cv::resize(im, im, cv::Size(48, 48));
            o[ind]         = cv_to_argb(im);
            const auto end = o[ind].end();
            for(auto ii = o[ind].begin(); ii != end; ++ii) {
               if(*ii == k_black) *ii = 0xff000000u;
            }

         } catch(std::exception& e) {
            LOG_ERR(
                format("FAILED to load icon for pose '{}', path={}, error={}",
                       str(pose),
                       path,
                       e.what()));
         }
      };

      load_image("pose-stand.png", PoseAnnotation::STAND);
      load_image("pose-walk.png", PoseAnnotation::WALK);
      load_image("pose-sit.png", PoseAnnotation::SIT);
      load_image("pose-lay.png", PoseAnnotation::LAY);
      load_image("pose-phone.png", PoseAnnotation::PHONE);
      load_image("pose-other.png", PoseAnnotation::OTHER);
      return o;
   };

   static const std::array<ARGBImage, n_pose_annotations()> icons
       = calc_icons();

   if(pose != PoseAnnotation::NONE) {
      const size_t ind = size_t(pose);
      const size_t ind0
          = (ind < icons.size() ? ind : size_t(PoseAnnotation::NONE));

      render_pose_icon(im, dcam, hist_xy, icons.at(ind0));
   }
}

void This::render_comp_graph(cv::Mat& im,
                             const int frame_no,
                             const tracks::ComputationData& comp_dat,
                             const bool render_labels) const noexcept
{
   check_image_(im);

   const AABB im_bounds = cv_im_bounds(im);
   const int im_w = im.cols, im_h = im.rows;
   const int radius
       = int(std::round(hist_X_to_image_strech_w(im) * 0.6 / hist_sz));

   auto project_to_hist = [&](const Vector3f& X) {
      return FloorHistogram::project_to_histogram_f(
          to_vec3(X), hist_bounds.top_left(), hist_sz);
   };

   auto render_op_s2d = [&](const auto& op_s2d) {
      if(op_s2d.t != frame_no) return;
      const auto& info     = *op_s2d.p2d_info_ptr;
      const auto& p2d      = *info.p2d_ptr;
      const bool is_interp = p2d.is_interpolation();
      if(is_interp) return;

      const uint32_t kolour = is_interp ? k_gray : colour_set_4(info.id);

      const Vector3f C = p2d.best_3d_result().Xs_centre();
      if(!C.is_finite()) return; // nothing to do

      const Vector2 X      = project_to_hist(C);
      const string label_s = render_labels ? format("{}", info.id) : ""s;

      render_p3d(im, X, kolour, label_s, p2d.theta(), true);
   };

   auto render_op_s2d_edges = [&](const auto& op_s2d) {
      if(op_s2d.t != frame_no) return;
      const auto& info     = *op_s2d.p2d_info_ptr;
      const auto& p2d      = *info.p2d_ptr;
      const bool is_interp = p2d.is_interpolation();
      if(is_interp) return;

      const Vector2 X0 = hist_X_to_image(
          im, project_to_hist(p2d.best_3d_result().Xs_centre()));
      // INFO(format("X0 = {}, has {} edges", str(X0), op_s2d.edges.size()));
      if(!X0.is_finite()) return;

      for(const auto& e : op_s2d.edges) {
         if(e.type != tracks::ComputationData::Edge::T0) continue;
         const auto o_idx = e.other_idx;
         Expects(size_t(o_idx) < comp_dat.pose_dat.size());
         const auto& o_info = *(comp_dat.pose_dat[o_idx].p2d_info_ptr);
         const Vector2 X1   = hist_X_to_image(
             im, project_to_hist(o_info.p2d_ptr->best_3d_result().Xs_centre()));
         if(!X1.is_finite()) continue;
         const auto k = normalized_score_to_colour(e.weight);
         // TRACE(format("DRAW to X1 = {} (id = {}-{}), wgt = {}",
         //              str(X1),
         //              info.id,
         //              o_info.id,
         //              e.weight));
         bresenham(X0, X1, im_bounds, [&](int x, int y) { set(im, x, y, k); });
      }
   };

   // // -- //
   // for(const auto& op_s2d : comp_dat.pose_dat) render_op_s2d(op_s2d);

   // // Render the graph edges
   // for(const auto& op_s2d : comp_dat.pose_dat) render_op_s2d_edges(op_s2d);

   auto render_xyt_node = [&](const auto& o) {
      Expects(o.t() == frame_no);
      const Vector3f Y     = Vector3f(o.x(), o.y(), 0.0f);
      const bool is_interp = o.is_interp();
      const int id         = o.frame_id();
      const float theta    = o.gaze_theta();

      const Vector2 X       = project_to_hist(Y);
      const string label_s  = render_labels ? format("{}", id) : ""s;
      const uint32_t kolour = colour_set_4(unsigned(id));

      if(!X.is_finite()) return; // nothing to do
      render_p3d(im, X, kolour, label_s, theta, !is_interp);
   };

   for(const auto& frame : comp_dat.frame_dat) {
      if(frame.t != frame_no) continue;
      for(const auto& o : frame.nodes) render_xyt_node(o);
   }
}

// ------------------------------------------------------------------ render-XYT
//
void This::render_XYT(cv::Mat& im,
                      const float x,
                      const float y,
                      const float theta,
                      const int id) const noexcept
{
   const uint32_t kolour = colour_set_4(unsigned(id));
   const string label    = format("{}", id);

   auto project_to_hist = [&](const Vector3& X) {
      return FloorHistogram::project_to_histogram_f(
          X, hist_bounds.top_left(), hist_sz);
   };

   const Vector2 X = project_to_hist(to_vec3(Vector3f(x, y, 0.0f)));
   render_p3d(im, X, kolour, label, theta, true);
}

} // namespace perceive::movie
