
#pragma once

#include "pose-skeleton-exec.hpp"

#include "perceive/geometry/human-heights.hpp"
#include "perceive/geometry/point-cloud.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
// Gaze-direction
// Height
// XY-circle overlap
// Radius
// Camera centers close enough

// --------------------------------------------------------------------- Pose 3D
//
struct Pose3D
{
 public:
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      real foot_dist_radius_factor   = 2.0; // times the ra
      real skeleton_disparity_weight = 2.0;
      bool feedback                  = false;
   };

   struct SensorPose
   {
      int16_t sensor_no{-1};
      int16_t detect_no{-1};
      shared_ptr<const Skeleton2D> pose;

      SensorPose() = default;
      SensorPose(int sensor_, int detect_, shared_ptr<const Skeleton2D> pose_)
          : sensor_no(int16_t(sensor_))
          , detect_no(int16_t(detect_))
          , pose(pose_)
      {
         Expects(pose_ != nullptr);
      }
      SensorPose(const SensorPose&) = default;
      SensorPose(SensorPose&&)      = default;
      ~SensorPose()                 = default;
      SensorPose& operator=(const SensorPose&) = default;
      SensorPose& operator=(SensorPose&&) = default;

      bool test_eq(const SensorPose& o, string& s, bool report) const noexcept;
      bool operator==(const SensorPose& o) const noexcept;
      bool operator!=(const SensorPose& o) const noexcept;

      size_t memory_usage() const noexcept;
      const EuclideanTransform&
      et(const SceneDescription& scene_desc) const noexcept;

      Json::Value to_json() const noexcept;
      void read(const Json::Value&) noexcept(false);
      string to_json_str() const noexcept;
   };

   static constexpr real k_pose_radius = 0.30;
   static constexpr real k_granularity = 0.025; // 2.5cm ~1 inch
   static constexpr real k_min_height  = 1.20;
   static constexpr real k_max_height  = 2.20;
   static constexpr int k_n_heights
       = int((k_max_height - k_min_height) / k_granularity);
   static constexpr real k_height_delta
       = (k_max_height - k_min_height) / (k_n_heights - 1);

   static constexpr real k_height(int ind) noexcept
   {
      return k_min_height + k_height_delta * ind;
   }

   static real prob_height(int ind) noexcept
   {
      real hgt = k_height(ind);
      real low = (ind == 0) ? 0.0 : (hgt - 0.5 * k_granularity);
      real hgh = (1 + ind == k_n_heights) ? 100.0 : (hgt + 0.5 * k_granularity);
      return human::prob_height_between(-1, -1, low, hgh);
   }

 private:
   // The same sensor-num_pose-id can appear in different poses.
   // THAT IS, we're dealing with soft constraints here.
   Params params_;
   vector<SensorPose> poses_;
   unsigned i_      = 0;
   real gaze_theta_ = dNAN; // gaze direction
   bool is_valid_   = false;

   array<Vector2f, k_n_heights> Cs_; // one for each height
   array<float, k_n_heights> ius_;   // one for each height

 public:
   Pose3D();
   Pose3D(const Pose3D&) = default;
   Pose3D(Pose3D&&)      = default;
   ~Pose3D()             = default;
   Pose3D& operator=(const Pose3D&) = default;
   Pose3D& operator=(Pose3D&&) = default;

   size_t memory_usage() const noexcept;

   // If '!C().is_finite()', then one of the poses is very poor quality
   const Params& params() const noexcept { return params_; }
   Vector3 C() const noexcept
   {
      return to_vec3(Vector3f(Cs_[i_].x, Cs_[i_].y, 0.0f));
   }
   real height() const noexcept { return k_height(int(i_)); }
   real radius() const noexcept { return k_pose_radius; }
   real gaze_theta() const noexcept { return gaze_theta_; }
   real iu() const noexcept { return real(ius_[i_]); }

   const vector<SensorPose>& poses() const noexcept { return poses_; }
   bool is_valid() const noexcept { return is_valid_; }
   Vector3 Cs(int ind) const noexcept
   {
      return to_vec3(Vector3f(Cs_[size_t(ind)].x, Cs_[size_t(ind)].y, 0.0f));
   }

   Vector2 gaze_vec2() const noexcept
   {
      return Vector2(cos(gaze_theta_), sin(gaze_theta_));
   }

   // After changing any 'iu', this function should be called
   // in order to update 'C', 'height', etc above
   void update_best_iu() noexcept;
   array<float, k_n_heights>& ius() noexcept { return ius_; }
   const array<float, k_n_heights>& ius() const noexcept { return ius_; }

   void init(const SceneDescription* scene_desc,
             std::function<const PointCloud*(int sensor_no)> get_point_cloud,
             const PoseSkeletonExec::Result* op_ret,
             const Params& params,
             const int sensor_num,
             const int openpose_detection_num) noexcept;

   bool test_eq(const Pose3D& o, const bool report) const noexcept;
   bool operator==(const Pose3D& o) const noexcept;
   bool operator!=(const Pose3D& o) const noexcept;

   // Combine results
   static Pose3D
   combine(const SceneDescription* scene_desc,
           std::function<const PointCloud*(int sensor_no)> get_point_cloud,
           const Params& params,
           vector<SensorPose>&& poses) noexcept;

   static Pose3D
   combine(const SceneDescription* scene_desc,
           std::function<const PointCloud*(int sensor_no)> get_point_cloud,
           const Params& params,
           const PoseSkeletonExec::Result* op_ret,
           const vector<Point2>& poses) noexcept;

   static Pose3D
   combine(const SceneDescription* scene_desc,
           std::function<const PointCloud*(int sensor_no)> get_point_cloud,
           const Params& params,
           const Pose3D& A,
           const Pose3D& B) noexcept;

   static Pose3D combine(const SceneDescription* scene_desc,
                         const Params& params,
                         const Pose3D& A,
                         const Pose3D& B) noexcept;

   // The {sensor number, pose}
   unsigned size() const noexcept { return unsigned(poses_.size()); }

   string to_string(const SceneDescription* scene_desc) const noexcept;

   // IO
   string to_json_string() const noexcept;
   Json::Value to_json() const noexcept;
   void read_with_defaults(const Json::Value& node,
                           const Pose3D* defaults = nullptr) noexcept(false);

   static Pose3D read(const Json::Value& node,
                      const Pose3D* defaults = nullptr) noexcept(false);
};

// get_point_cloud returns nullptr if the sensor isn't the reference sensor
Pose3D
make_pose_3d(const SceneDescription* scene_desc,
             std::function<const PointCloud*(int sensor_no)> get_point_cloud,
             const PoseSkeletonExec::Result* op_ret,
             const Pose3D::Params& params,
             const int sensor_num,
             const int openpose_detection_num) noexcept;

real sensor_pose_score(
    const SceneDescription& scene_desc,
    const vector<const Pose3D::SensorPose*> sensor_pose_ptrs) noexcept;

META_READ_WRITE_LOAD_SAVE(Pose3D)

} // namespace perceive
