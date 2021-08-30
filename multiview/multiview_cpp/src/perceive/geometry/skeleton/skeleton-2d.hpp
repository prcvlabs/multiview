
#pragma once

#include "bone.hpp"
#include "skeleton-3d.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/cylinder.hpp"
#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
struct Skeleton2D final
{
   static constexpr int k_n_keypoints = skeleton::k_n_keypoints;

   enum PoseModel : int { BODY_25, COCO_18, MPI_15, MPI_15_4 };
   static PoseModel int_to_pose_model(int) noexcept;
   friend const char* str(const PoseModel) noexcept;

   struct Params final
   {};

   struct Keypoint final
   {
      int32_t part = 0; // KeypointName depends on the model
      Vector2f pos = Vector2f::nan();
      float score  = 0.0f;

      bool operator==(const Keypoint&) const noexcept;
      bool operator!=(const Keypoint&) const noexcept;

      float operator[](int ind) const noexcept { return pos[ind]; }
      bool is_valid() const noexcept { return pos.is_finite(); }
      const Vector2f& xy() const noexcept { return pos; }
   };

   struct CylinderResult
   {
      // cylinder "near" and "far" keypoints, in order of
      // `get_p2d_proprotions()`
      vector<std::pair<Vector3, Vector3>> keypoints;
      real height_stddev = dNAN;           // stddev in height estimates
      real theta         = dNAN;           // gaze angle
      Plane4f p3         = Plane4f::nan(); // Calculated from torso_p3_.xyz()
      int sensor_no      = -1;
      Cylinder Cy; // Height is in the cylinder itself
   };

 private:
   PoseModel model_                               = BODY_25; //
   int frame_no_                                  = -1;      //
   int sensor_no_                                 = -1;      //
   std::array<Keypoint, k_n_keypoints> keypoints_ = {};      //
   Vector3f cam_centre_                           = {};      //
   std::array<Vector3f, k_n_keypoints> rays_      = {};      // in 3D space
   Vector3f centre_ray_                           = {};      //
   real theta_                                    = 0.0;     // gaze angle
   real score_                                    = 0.0;     //
   std::array<Vector2f, 2> eigen_vecs_            = {};      // of the 2D model
   ProjectiveFloorCylinder cylinder_              = {};
   Skeleton3D best_3d_result_                     = {};
   bool is_interpolation_                         = false;

 public:
   Skeleton2D()                      = default;
   Skeleton2D(const Skeleton2D&)     = default;
   Skeleton2D(Skeleton2D&&) noexcept = default;
   ~Skeleton2D()                     = default;
   Skeleton2D& operator=(const Skeleton2D&) = default;
   Skeleton2D& operator=(Skeleton2D&&) noexcept = default;

   void init(const Params& params,
             const int frame_no,
             const int sensor_no,
             const DistortedCamera* dcam_ptr, // could be nullptr
             const PoseModel model,
             const vector<Keypoint>& keypoints);

   bool test_eq(const Skeleton2D& o, string& s, bool report) const noexcept;
   bool operator==(const Skeleton2D& o) const noexcept;
   bool operator!=(const Skeleton2D& o) const noexcept;

   size_t memory_usage() const noexcept;

   static Skeleton2D interpolate(const Params& params,
                                 const Skeleton2D& A,
                                 const Skeleton2D& B,
                                 const int frame_no,
                                 const DistortedCamera* dcam_ptr);

   AABB aabb() const noexcept; // calculate the bounding box
   AABB head_aabb() const noexcept;
   AABB torso_aabb() const noexcept;

   vector<LABImage> make_image_patches(const int patch_w,
                                       const int patch_h,
                                       const LABImage&) const noexcept(false);

   // Getters
   bool is_interpolation() const noexcept
   {
      // TRACE("BEFORE");
      return is_interpolation_;
      // TRACE("AFTER");
   }
   int frame_no() const noexcept { return frame_no_; }
   int sensor_no() const noexcept { return sensor_no_; }
   PoseModel model() const noexcept { return model_; }
   const auto& keypoints() const noexcept { return keypoints_; }
   real theta() const noexcept { return theta_; }
   real score() const noexcept { return score_; }
   const auto& cam_centre() const noexcept { return cam_centre_; }
   const auto& rays() const noexcept { return rays_; }
   const Vector3f& centre_ray() const noexcept { return centre_ray_; }
   const Vector3f& center_ray() const noexcept { return centre_ray_; }
   const std::array<Vector2f, 2>& eigen_vecs() const noexcept
   {
      return eigen_vecs_;
   }
   const Plane4f& torso_p3() const noexcept;

   const auto& best_3d_result() const noexcept { return best_3d_result_; }

   const ProjectiveFloorCylinder& proj_cylinder() const noexcept
   {
      return cylinder_;
   }
   CylinderResult realize_cylinder(const DistortedCamera& dcam,
                                   const real height,
                                   const bool feedback = false) const noexcept;
   CylinderResult realize_cylinder(const DistortedCamera& dcam,
                                   const Vector3& X,
                                   const bool feedback = false) const noexcept;

   Vector2 keypoint_position(skeleton::KeypointName kp) const noexcept;
   const Vector3f& keypoint_ray(skeleton::KeypointName kp) const noexcept;

   void set_keypoint(skeleton::KeypointName kp,
                     const Vector2f& xy,
                     float score) noexcept;

   // These functions return "NAN" keypoints weren't detected/don't exist
   const Vector2f& nose() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::NOSE)].pos;
   }
   const Vector2f& neck() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::NOTCH)].pos;
   }
   const Vector2f& pelvis() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::PELVIS)].pos;
   }
   const Vector2f& l_eye() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_EYE)].pos;
   }
   const Vector2f& r_eye() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_EYE)].pos;
   }
   const Vector2f& l_ear() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_EAR)].pos;
   }
   const Vector2f& r_ear() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_EAR)].pos;
   }
   const Vector2f& l_shoulder() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_SHOULDER)].pos;
   }
   const Vector2f& r_shoulder() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_SHOULDER)].pos;
   }
   const Vector2f& l_elbow() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_ELBOW)].pos;
   }
   const Vector2f& r_elbow() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_ELBOW)].pos;
   }
   const Vector2f& l_wrist() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_WRIST)].pos;
   }
   const Vector2f& r_wrist() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_WRIST)].pos;
   }
   const Vector2f& l_hip() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_HIP)].pos;
   }
   const Vector2f& r_hip() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_HIP)].pos;
   }
   const Vector2f& l_knee() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_KNEE)].pos;
   }
   const Vector2f& r_knee() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_KNEE)].pos;
   }
   const Vector2f& l_ankle() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_ANKLE)].pos;
   }
   const Vector2f& r_ankle() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_ANKLE)].pos;
   }
   const Vector2f& l_heal() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_HEAL)].pos;
   }
   const Vector2f& r_heal() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_HEAL)].pos;
   }
   const Vector2f& l_big_toe() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_BIG_TOE)].pos;
   }
   const Vector2f& r_big_toe() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_BIG_TOE)].pos;
   }
   const Vector2f& l_small_toe() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::L_SMALL_TOE)].pos;
   }
   const Vector2f& r_small_toe() const noexcept
   {
      return keypoints_[int(skeleton::KeypointName::R_SMALL_TOE)].pos;
   }

   Vector2f head() const noexcept; // nose, eyes, ears in that order
   Vector2f chest() const noexcept { return 0.5f * (neck() + pelvis()); }
   Vector2f feet() const noexcept; // average of left and right ankle/feet
   AABBi head_roi() const noexcept;

   // -- IO --
   Json::Value to_json() const noexcept;
   void read(const Json::Value&) noexcept(false);
   string to_string() const noexcept;
   string to_json_str() const noexcept;
   friend string str(const Skeleton2D& pose) noexcept;
};

string str(const vector<Skeleton2D>& poses) noexcept;

Skeleton2D::PoseModel to_pose_model(const string_view val) noexcept(false);

vector<vector<LABImage>>
parallel_make_patches(const int patch_w,
                      const int patch_h,
                      const vector<shared_ptr<const Skeleton2D>>& p2ds,
                      std::function<const LABImage*(int sensor_no)> get_lab);

} // namespace perceive
