
#pragma once

#include "json/json.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/scene/aruco-result-info.hpp"
#include "perceive/scene/scene-description-info.hpp"
#include "perceive/scene/scene-manifest.hpp"

#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/timestamp.hpp"

#include "perceive/graphics/sprite.hpp"

#ifdef USING_OPENGL
#include "gl/gl-texture.hpp"
#include "gl/gl-z-buffer.hpp"
#endif

namespace perceive
{
// 3D model file (optional)
// Set of camera files
struct SceneDescription
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;
   void unpack_raw_images(bool is_advance_one_frame, int scene_frame_no);
   vector<cv::Mat> grab_video_frame_(int scene_frame_no) const noexcept;
   std::vector<CachingUndistortInverse> cu_;
   unsigned int background_tex_{0};

   vector<ARGBImage> get_floor_grids() const noexcept;

 public:
   CUSTOM_NEW_DELETE(SceneDescription)

   SceneDescription();
   ~SceneDescription();

   SceneDescription(const SceneDescription&) = delete; // copy
   SceneDescription& operator=(const SceneDescription&) = delete;

   SceneDescription(SceneDescription&&) = default; // move
   SceneDescription& operator=(SceneDescription&&) = default;

   struct InitOptions
   {
      DataSource source           = DataSource::DEFAULT;
      real target_fps             = 0.0;
      string optional_search_path = ""s;
      bool on_the_fly_still       = false;
      bool no_stereo              = false;
      bool verbose                = false;
   };

   // ---- Functions ---- //
   void init(const SceneManifest&, const InitOptions& opts) noexcept(false);
   void init_from_json(const Json::Value&,
                       const InitOptions& opts) noexcept(false);

   int n_cameras() const noexcept { return int(filenames.size()); }
   int n_sensors() const noexcept { return int(sensor_ids.size()); }

   int find_camera(const string_view key) const noexcept;
   int sensor_lookup(int bcam_index, int index) const noexcept;
   Point2 bcam_lookup(int sensor_index) const noexcept; // bcam-index, index
   std::pair<int, int>
   bcam_lookup2(int sensor_index) const noexcept; // bcam-index, index
   int n_sensors_for(int bcam_index) const noexcept { return 2; }
   int sensor_position(int sensor_index) const noexcept;

   Timestamp current_timestamp() const noexcept;
   bool has_current_frame() const noexcept;
   int current_frame_no() const noexcept;
   int n_frames() const noexcept;

   string to_string() const noexcept;

   bool advance_one_frame_();      // thread-safe with 'get_images_for_frame'
   bool seek_frame_(int frame_no); // thread-safe with 'get_images_for_frame'
   bool is_no_stereo() const noexcept;

   struct FrameImages
   {
      bool frame_loaded{false}; // FALSE if no frame was loaded
      int frame_no{0};
      vector<cv::Mat> raw_images;    // Distorted images, one per camera.
      vector<cv::Mat> sensor_images; // distorted, one per sensor
      vector<Vec3fImage> labs;
   };
   FrameImages get_images_for_frame(int frame_no) const noexcept; // thread-safe

   // Every frame that's read is accumulated into buffer... one sensor
   // per camera. (i.e., sensor zero). Returns the average image.
   Vec3fImage accum_blurred_LAB(int sensor_no) const noexcept;
   ARGBImage argb_still(int cam_no) const noexcept;
   LABImage LAB_still(int cam_no) const noexcept;
   LABImage sensor_LAB_still(int sensor_ind) const noexcept;

   // ---- Members ---- TREAT AS CONSTANTS ---- //
   SceneDescriptionInfo scene_info; // .
   Timestamp epoch;                 // Timestamp the identifies epoch
   Timestamp timestamp0;            // Timestamp for logical beginning.
   real scene_fps{0.0};             // fps for scene
   real frame_duration() const noexcept { return 1.0 / scene_fps; }

   // This is "one-per-camera"
   vector<string> filenames;       // absolute path, one per camera.
   vector<bool> is_video;          // TRUE only for videos.
   vector<cv::Mat> raw_images;     // Distorted images, one per camera.
   vector<cv::Mat> raw_prev_frame; // Distorted images for previous camera

   vector<cv::Mat> ref_disparities; // always left image is ref. May be empty.

   vector<BinocularCameraInfo> bcam_infos;    // binocular camera info
   vector<EuclideanTransform> cam_transforms; // optimized
   vector<vector<DistortedCamera>> dcams;     // convenient to work with
   const DistortedCamera& dcam(int sensor_ind) const noexcept
   {
      const auto [i, j] = bcam_lookup2(sensor_ind);
      Expects(size_t(i) < dcams.size());
      Expects(size_t(j) < dcams[size_t(i)].size());
      return dcams[size_t(i)][size_t(j)];
   }

   // This is "one-per-sensor"
   vector<string> sensor_ids;                    // the order of sensor ids.
   vector<cv::Mat> sensor_image;                 // distorted, one per sensor
   vector<cv::Mat> sensor_previous;              // distorted, previous frame
   vector<EuclideanTransform> sensor_transforms; // optimized.
   vector<EuclideanTransform> aruco_transforms;  // may be empty.

   const ARGBImage& floor_grid(int sensor_ind) const noexcept;
   const vector<ARGBImage>& all_floor_grids() const noexcept;

   // The background-image, if it exists
   ARGBImage background_image; // This is an OpenGL image
   bool has_background_image() const noexcept;
#ifdef USING_OPENGL
   // We put the OpenGL resource here.
   ARGBImage make_background_image_GL() const noexcept;
   ARGBImage background_image_GL;
   real background_image_h_tex_coord{1.0};
   real background_image_w_tex_coord{1.0};
   unsigned int background_tex_id() const noexcept;
#endif

   // Depth buffers made from rendering CAD model from sensor positions
   static constexpr auto fbo_w    = 1200;
   static constexpr auto fbo_h    = 600;
   static constexpr auto fbo_hfov = 70.0 * (M_PI / 180.0); // Radians
   struct ZBuffer
   {
#ifdef USING_OPENGL
      GlProjection port;
      GlZBuffer z_buffer;
#endif
   };
   vector<ZBuffer> depth_buffers;

   const DistortionModel& model(int sensor_ind) const noexcept
   {
      auto x = bcam_lookup(sensor_ind);
      return bcam_infos[size_t(x.x)].M[size_t(x.y)];
   }

   const CachingUndistortInverse& cu(int sensor_ind) const noexcept
   {
      return cu_[size_t(sensor_ind)];
   }

   string cad_model_key;
   shared_ptr<const Sprite> cad_model_ptr; // CAD model.

   friend string str(const SceneDescription& o) { return o.to_string(); }
};

shared_ptr<SceneDescription>
load_scene_from_manifest(const Json::Value& data,
                         const SceneDescription::InitOptions& opts) noexcept;

void export_movie_and_manifest(const SceneDescription&,
                               const string_view outdir,
                               const real frame_rate,
                               const int start_frame = 0,
                               const int n_frames    = -1);

vector<Point2> calc_frame_pairs(const real in_fps,
                                const real target_fps,
                                const int N) noexcept(false);

int calc_movie_frame_offset(const real movie_fps,
                            const Timestamp& timestamp0,
                            const Timestamp& movie_timestamp) noexcept;

} // namespace perceive
