
#pragma once

#include "stdinc.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-3d.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/geometry/skeleton/skeleton-2d-info.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
//
// All tracker data must be contained within this object.
//
struct LocalizationData
{
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      // Motion histogram
      bool use_median{false};
      real n_deviations{3.0};

      //
      real theta_stddev_degrees = 4.0; // degrees (!)

      // appearance score
      real person_diameter   = 0.400; // 40cm across the shoulders
      real background_median = 0.000; // background
      real background_stddev = 20.00;
      real track_median      = 200.0; // appearance median
      real track_stddev      = 50.00; //        and spread
      real prob_bg_max       = 0.050;

      // weight = max(0.0, 1.0 - factor * dist / radius)
      real dist_weight_factor = 1.0;

      // In this method we:
      // [1] take each openpose detection (per sensor)
      // [2] at each possible height, find the image in the histogram
      //     and multiply by the probability that it's foreground.
      //     (stored as a vector in the Pose3D)
      // [3] reweight the vector so that it adds to 1.0
      // [4] draw the vector onto the final localization, using
      //     the distance-weight factor
      // --- Alternative
      // Localize by Skeleton3D + Sparse Histograms
      bool prefer_openpose_method{true};
      real prob_shift = 0.05;
      real prob_scale = 2.0;  // scale * (p - shift)
      real zero_value = -0.2; // Higher the better

      // Final bounds for localization score
      real localization_min{-20.0};
      real localization_max{20.0};
      real filter_radius{0.05}; // in meters

      Vector4f false_positive_weight_vector = {0.25f,  // p2d-score
                                               0.25f,  // cloud-in-bounds height
                                               0.00,   // in-bounds height
                                               0.25f}; // p2d-shape

      unsigned pose2d_patch_w = 12;
      unsigned pose2d_patch_h = 3;

      // This parameter sets the sensitivity of the histogram to
      // extrinsic calibration errors. It means: if the value is
      // 'X', then we accept 1 radius of reprojection error at
      // 'X' metres.
      float extrinsic_calibration_error_factor = 4.0f;

      real loc_min() const noexcept
      {
         return std::min(localization_min, localization_max);
      }
      real loc_max() const noexcept
      {
         return std::max(localization_min, localization_max);
      }

      Pose3D::Params p3d_params;
      bool apply_smooth_mhist_to_p3ds = true;
   };

   Grey16Image loc_hist = {}; // localization histogram
   float loc_min        = 0.0;
   float loc_max        = 1.0;
   int frame_no         = -1;
   real hist_sz         = 0.0;
   AABB bounds          = {}; // Copied in from FloorHistogram parameters
   vector<vector<Skeleton2DInfo>> p2ds = {}; // One p2d per sensor

   bool operator==(const LocalizationData& o) const noexcept;
   bool operator!=(const LocalizationData& o) const noexcept;

   size_t memory_usage() const noexcept;

   bool invariant_check() const noexcept;

   string to_string() const noexcept;

   friend string str(const LocalizationData&) noexcept;

   // Human heights [0.75..2.13], toddler to 7' tall
   static constexpr real k_min_height = 0.00; // A toddler
   static constexpr real k_max_height = 3.00; // 7' tall

   bool in_bounds(int x, int y) const noexcept;
   float hist_cost(int x, int y) const noexcept;
   Vector2 project_to_hist(const Vector3& X) const noexcept;

   // 'X' is in metres
   template<typename T> float hist_cost_X(const T& X) const noexcept
   {
      return scale_01(hist_cost_X_01(X));
   }

   template<typename T> float hist_cost_X_01(const T& X) const noexcept
   {
      const auto xy = project_to_hist(Vector3(real(X(0)), real(X(1)), 0.0));
      const int x   = int(xy.x + 0.499);
      const int y   = int(xy.y + 0.499);
      if(!loc_hist.in_bounds(x, y)) return 1.0f;
      constexpr float max_val
          = float(std::numeric_limits<Grey16Image::value_type>::max());
      constexpr float max_val_inv = 1.0f / max_val;
      return loc_hist(x, y) * max_val_inv;
   }

   // Scale a value in [0..1] to [loc-min..loc-max]
   float scale_01(const float val) const noexcept
   {
      return std::clamp<float>(
          (val * (loc_max - loc_min)) + loc_min, loc_min, loc_max);
   }

   // Read/write to/from a stream
   void read(FILE* fp) noexcept(false);
   void write(FILE* fp) const noexcept;

   // We want to include other data here...

   // @param openexec_ptr could be null
   // @param get_point_cloud returns the PointCloud for 'sensor-no', or
   //                        nullptr if not the primary sensor.
   static LocalizationData
   calculate(const SceneDescription& scene_desc,
             const PoseSkeletonExec::Result* openpose_ret_ptr,
             const FloorHistogram& floor_hist,
             std::function<const PointCloud*(int sensor_no)> get_point_cloud,
             std::function<const ImageFeatures2d*(int sensor_no)> get_f2d,
             std::function<const BinocularCamera*(int cam_no)> get_bcam,
             const LocalizationData::Params& p,
             const bool feedback,
             const int frame_no) noexcept;
};

// ------------------------------------------------------------------- load/save
//
struct LocalizationDataEnvelope
{
   unsigned start_frame = 0;
   real frame_duration  = dNAN;
   vector<LocalizationData> loc_data;
   size_t memory_usage() const noexcept
   {
      size_t ret = sizeof(decltype(*this));
      for(const auto& ldat : loc_data) ret += ldat.memory_usage();
      return ret;
   }
};
LocalizationDataEnvelope load_localization_data(FILE* fp) noexcept;
LocalizationDataEnvelope
load_localization_data(const string_view filename) noexcept(false);

void save_localization_data(
    FILE* fp,
    const unsigned start_frame,
    const unsigned n_frames,
    const real frame_duration,
    std::function<const LocalizationData*(unsigned)> get_loc,
    const bool verbose) noexcept;

void save_localization_data(
    const string_view filename,
    const unsigned start_frame,
    const unsigned n_frames,
    const real frame_duration,
    std::function<const LocalizationData*(unsigned)> get_loc,
    const bool verbose) noexcept(false);

// -----------------------------------------------------------------------------
//
FloatImage make_motion_histogram(const FloatImage& hist,
                                 const bool use_median,
                                 const real n_deviations) noexcept;

FloatImage make_bg_histogram(const FloatImage& motion_hist,
                             const LocalizationData::Params& p,
                             const real hist_sz) noexcept;

FloatImage make_fg_3d_histogram(const FloatImage& motion_hist,
                                const LocalizationData::Params& p,
                                const real hist_sz) noexcept;

FloatImage make_fg_histogram(const FloatImage& motion_hist,
                             const LocalizationData::Params& p,
                             const real hist_sz) noexcept;

FloatImage make_fg_histogram(const FloatImage& fg0,
                             const FloatImage& fg1,
                             const LocalizationData::Params& p) noexcept;

std::tuple<FloatImage, FloatImage, vector<vector<Skeleton2DInfo>>>
make_fg_openpose_histogram(
    const SceneDescription& scene_desc,
    const PoseSkeletonExec::Result& openpose_ret,
    const FloorHistogram& floor_hist,
    std::function<const PointCloud*(int sensor_no)> get_point_cloud,
    std::function<const ImageFeatures2d*(int sensor_no)> get_f2d,
    std::function<const BinocularCamera*(int cam_no)> get_bcam,
    const FloatImage& motion_hist,
    const LocalizationData::Params& p) noexcept;

FloatImage make_fowlkes_histogram(const FloatImage& bg,
                                  const FloatImage& fg,
                                  const LocalizationData::Params& p,
                                  const real hist_sz) noexcept;

} // namespace perceive
