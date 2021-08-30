
#pragma once

#include "render-tracks.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/pipeline/test-output.hpp"
#include "perceive/scene/pose-annotation.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/opencv-helpers.hpp"

namespace perceive
{
struct PipelineOutput;
struct LocalizationData;
class Tracklet;
class Tracks;
struct Track;
struct TrackPoint;
struct FowlkesResult;
struct SceneDescription;

namespace tracks
{
   struct ComputationData;
}

} // namespace perceive

namespace perceive::movie
{
void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const int sensor_no,
                const vector<Pose3D>& p3ds) noexcept;

void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const vector<const Skeleton2D*>& poses) noexcept;

void blur_faces(cv::Mat& im,
                const int frame_no,
                const bool draw_pose,
                const vector<shared_ptr<const Skeleton2D>>& poses) noexcept;

void blur_faces(
    cv::Mat& im,
    const int frame_no,
    const bool draw_pose,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept;

void blur_faces(
    ARGBImage& argb,
    const int frame_no,
    const bool draw_pose,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept;

// Outputs {[label-position, labels], ...}
vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<Pose3D>& p3ds) noexcept;

vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<shared_ptr<const Skeleton2D>>& p2ds) noexcept;

vector<std::pair<Point2, string>> render_poses(
    ARGBImage& argb,
    const int sensor_no,
    const unsigned n_poses,
    std::function<const Skeleton2D*(unsigned idx)> get_pose_ptr) noexcept;

vector<std::pair<Point2, string>>
render_poses(ARGBImage& argb,
             const int sensor_no,
             const vector<Skeleton2DInfo>& p2d_infos) noexcept;

cv::Mat
resize_and_render_labels(const cv::Mat& im,
                         const vector<std::pair<Point2, string>>& labels,
                         unsigned new_w,
                         unsigned new_h) noexcept;

// Blend 'B' (foreground) onto 'A' (background) using 'alpha'
void blend_images(ARGBImage& A, const ARGBImage& B, const float alpha) noexcept;

struct MovieUtils
{
   unsigned hist_w = 0;
   unsigned hist_h = 0;
   AABB hist_bounds;
   real hist_sz    = dNAN;
   bool do_rotate  = false;
   real p3d_radius = 0.2;

   MovieUtils(unsigned hist_w_ = 0,
              unsigned hist_h_ = 0,
              AABB bounds      = {},
              real hist_sz_    = 0.0,
              bool do_rotate_  = false,
              real p3d_radius_ = 0.2)
       : hist_w(hist_w_)
       , hist_h(hist_h_)
       , hist_bounds(bounds)
       , hist_sz(hist_sz_)
       , do_rotate(do_rotate_)
       , p3d_radius(p3d_radius_)
   {}

   // @{ Working with histograms
   ARGBImage make_empty_hist() const noexcept;

   real hist_X_to_image_strech_w(const cv::Mat& im) const noexcept;
   real hist_X_to_image_strech_h(const cv::Mat& im) const noexcept;
   Vector2 hist_X_to_image(const cv::Mat& im, const Vector2& X) const noexcept;
   Vector2 world_X_to_image(const cv::Mat& im, const Vector3& X) const noexcept;
   void label_tracks(cv::Mat& im,
                     const int frame_no,
                     const vector<Track>& tracks) const noexcept;

   // The floor grid
   void render_grid(cv::Mat& im, const AABB& aabb) const noexcept;

   // A 'p3d' is  a track
   void render_p3d(cv::Mat& im,
                   const Vector2& X, // hist position
                   const uint32_t kolour,
                   const string_view label,
                   const float view_direction,
                   const bool fill,
                   const uint32_t gaze_kolour = 0) const noexcept;

   void
   render_tp(cv::Mat& im, const int id, const TrackPoint& tp) const noexcept;

   void render_track(cv::Mat& im,
                     const int frame_no,
                     const vector<tracks::NodeSequence>& seqs) const noexcept;

   void render_track(cv::Mat& im,
                     const int frame_no,
                     const vector<Track>& tracks) const noexcept;

   void render_track_match(cv::Mat& im,
                           const unsigned frame_no,
                           const pipeline::TestOutput& compare) const noexcept;

   void render_p2ds(cv::Mat& im,
                    const LocalizationData& ldat,
                    const bool render_labels,
                    const bool render_info_ids,
                    const bool fill_p2ds = true,
                    const bool draw_gaze = true) const noexcept;

   void render_comp_graph(cv::Mat& im,
                          const int frame_no,
                          const tracks::ComputationData& comp_dat,
                          const bool render_labels) const noexcept;

   void render_merged(cv::Mat& im, const vector<Pose3D>& p3ds) const noexcept;
   // @}

   void render_frame_no(cv::Mat& im, const unsigned frame_no) const noexcept;

   void render_trackpoint(ARGBImage& im,
                          const DistortedCamera& dcam,
                          const int track_id,
                          const TrackPoint& tp,
                          const bool render_icon) const noexcept;

   void render_gt_trackpoint(ARGBImage& im,
                             const DistortedCamera& dcam,
                             const TrackPoint& tp,
                             const bool render_icon) const noexcept;

   void render_pose_icon(ARGBImage& im,
                         const DistortedCamera& dcam,
                         const Point2& hist_xy,
                         const ARGBImage& icon) const noexcept;

   void render_pose_icon(ARGBImage& im,
                         const DistortedCamera& dcam,
                         const Point2& hist_xy,
                         const PoseAnnotation pose) const noexcept;

   void render_XYT(cv::Mat& im,
                   const float x,
                   const float y,
                   const float theta,
                   const int id) const noexcept;

 private:
   void check_image_(cv::Mat& im) const noexcept;
};

} // namespace perceive::movie
