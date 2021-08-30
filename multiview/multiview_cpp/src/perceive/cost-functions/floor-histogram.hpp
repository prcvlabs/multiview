
#pragma once

#include "stdinc.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/sparse-hist-cell.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
struct MovieStatsFile;

class FloorHistogram
{
 public:
   enum WeightingMethod : int8_t { WGT_NONE, WGT_NORM, WGT_QUADRANCE };

   // -- Parameters --
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      float hist_sz                 = 0.10f; // In meters
      float hist_min_z              = 0.50f;
      float hist_max_z              = 2.00f;
      float min_hist_count          = 0.0f;
      float max_hist_count          = 1000.0f;
      float gaussian_sigma          = 0.50f;
      WeightingMethod weighting     = WGT_NORM;
      bool apply_slic_still_weights = true;
      vector<unsigned> skip_cameras = {};

      // Minimum 'z' when calculating superpixel maximal labels
      float super_pixel_min_z = 0.25f;

      // Region of interest (could be NAN, min==max for none)
      // Vector2 bbox_min{4.0, -1.0}; // entrance dataset
      // Vector2 bbox_max{10.5, 14.0};
      Vector2 bbox_min{0.0, 0.0};
      Vector2 bbox_max{0.0, 0.0};

      // For calculating the motion histogram
      bool use_median = true; // median or average
      float n_spread  = 3.0f; // absdev or stddev

      // Expensive to calculate
      bool color_histogram = false; // Also outputs floor grid
      float color_hist_sz  = 0.01f;

      // std::string to_string() const;
   };

   AABB bounds     = {};
   real hist_sz    = 0.05;
   FloatImage hist = {};

   struct CameraInfo
   {
      FloatImage hist = {}; // `hist` above is the sum of all of these hists

      // Maps a super-pixel label for camera, to it's (maximal) hist cell,
      // or {-1, -1} if none.
      vector<SparseHistCell> slic_to_hist_xy = {};
   };
   vector<CameraInfo> camera_info; // camera_info.size() == n_cameras()

   // May be empty
   ARGBImage color_hist = {};
   real color_hist_sz   = 0.01;

   ARGBImage make_image(const bool gridlines = false) const;

   Vector2 project_to_histogram(const Vector3& Y, const real hist_sz) const
   {
      return project_to_histogram(Y, Vector2(bounds.left, bounds.top), hist_sz);
   }

   static Vector2 project_to_histogram(const Vector3& Y,
                                       const Vector2& top_left,
                                       const real hist_sz) noexcept
   {
      return project_to_histogram_f(Y, top_left, hist_sz);
   }

   static Vector2 project_to_histogram_f(const Vector3& Y,
                                         const Vector2& top_left,
                                         const real hist_sz) noexcept
   {
      return Vector2((Y.x - top_left.x) / hist_sz + 1e-9,
                     (Y.y - top_left.y) / hist_sz + 1e-9);
   }

   template<typename Point, typename Vec>
   static Vector2 unproject_hist_xy(const Point& xy,
                                    const Vec& top_left,
                                    const real hist_sz) noexcept
   {
      return Vector2(xy.x * hist_sz + top_left.x, xy.y * hist_sz + top_left.y);
   }

   template<typename Point, typename Vec>
   static Vector3 unproject_hist_xyz(const Point& xy,
                                     const Vec& top_left,
                                     const real hist_sz) noexcept
   {
      const auto X = unproject_hist_xy(xy, top_left, hist_sz);
      return Vector3(X.x, X.y, 0.0);
   }

   // -- Calculate --

   static bool calculate(const SceneDescription& scene_desc,
                         const Params& p,
                         AABB aabb,
                         unsigned n_cams,
                         std::vector<const BinocularCameraInfo*>& bcams,
                         std::vector<const EuclideanTransform*>& transforms,
                         std::vector<const PointCloud*>& point_clouds,
                         std::vector<const cv::Mat*>& rect_images,
                         std::vector<const IntImage*>& slic_lookups,
                         std::vector<const ImageFeatures2d*>& features,
                         std::vector<Vector3>& translations,
                         FloorHistogram& floor_hist,
                         std::function<bool()> is_canceled,
                         const bool feedback,
                         const string_view outdir);

   static FloatImage make_motion_histogram(const Params& p,
                                           const FloatImage& hist);
};

string str(FloorHistogram::WeightingMethod x) noexcept;

FloorHistogram::WeightingMethod
to_floor_hist_weighting_method(const string_view) noexcept(false);

GreyImage make_hist_image(const FloatImage& im);

BinaryImage make_entrance_zone(const vector<Vector2>& region,
                               const Vector2& top_left, // in meters
                               const unsigned w,        // width and height
                               const unsigned h,        // of histogram
                               const real hist_sz);     // histogram cell size

void find_entrance_region(const vector<Vector2>& region,
                          const Vector2& top_left, // in meters
                          const unsigned w,        // width and height
                          const unsigned h,        // of histogram
                          const real hist_sz,
                          std::function<void(int, int)> f);

void draw_entrance_region(ARGBImage& im,
                          const vector<Vector2>& region,
                          const Vector2 top_left,
                          const unsigned width,
                          const unsigned height,
                          const real hist_sz,
                          const uint32_t kolour,
                          const real alpha);

void draw_entrance_region(ARGBImage& im,
                          const vector<Vector2>& region,
                          const FloorHistogram& floor_hist,
                          const real hist_sz,
                          const uint32_t kolour,
                          const real alpha = 0.5);

void draw_gridlines(ARGBImage& im,
                    const AABB& bounds,
                    const real hist_sz,
                    const uint32_t kolour,
                    const real alpha = 0.5);

void update_floor_hist_cam_p3_distance(
    FloatImage& dist_im,
    const AABB& bounds,
    const real hist_sz,
    const EuclideanTransform& cam_et) noexcept;

} // namespace perceive
