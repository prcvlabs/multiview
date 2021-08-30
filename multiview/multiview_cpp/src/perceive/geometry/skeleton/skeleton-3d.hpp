
#pragma once

#include "bone.hpp"

#include "perceive/geometry/vector.hpp"

namespace perceive
{
struct DistortedCamera;
}

namespace perceive
{
struct Skeleton3D
{
 private:
   Vector3f C_          = Vector3f::nan(); // camera center
   int sensor_no_       = -1;
   Plane4f torso_p3_    = Plane4f::nan(); // pelvis-shoulder direction
   Vector3f up_n_       = Vector3f::nan();
   Vector3f left_n_     = Vector3f::nan();
   Vector3f hip_n_      = Vector3f::nan(); // hip-pelvis-hip plane normal
   Vector3f head_n_     = Vector3f::nan(); // where the head is looking
   Vector3f X_          = Vector3f::nan(); // "cylinder" base
   Vector3f floor_line_ = Vector3f::nan(); // line on the floor (C_..X_)
   Vector2f floor_ray_  = Vector2f::nan();

   // 3D recovery for a figure of average height
   float height_ = fNAN;
   vector<Vector3f> Xs_;     // recovered points
   float seconds_    = fNAN; // seconds to generate
   float reproj_err_ = 0.0f;

 public:
   Skeleton3D();
   Skeleton3D(const Skeleton3D&) = default;
   Skeleton3D(Skeleton3D&&)      = default;
   ~Skeleton3D()                 = default;
   Skeleton3D& operator=(const Skeleton3D&) = default;
   Skeleton3D& operator=(Skeleton3D&&) = default;

   static Skeleton3D fit_3d_ret(const DistortedCamera* dcam_ptr,
                                const Skeleton2D& p2d,
                                const bool feedback) noexcept(false);

   bool is_valid() const noexcept { return X_.is_finite(); }
   const auto& C() const noexcept { return C_; }
   const auto& torso_p3() const noexcept { return torso_p3_; }
   const auto& up_n() const noexcept { return up_n_; }
   const auto& forward_n() const noexcept { return torso_p3_.xyz(); }
   const auto& left_n() const noexcept { return left_n_; }
   const auto& hip_n() const noexcept { return hip_n_; }
   const auto& head_n() const noexcept { return head_n_; }
   float height() const noexcept { return height_; }
   float radius(float calc_height = fNAN) const noexcept
   {
      constexpr auto r = 0.5f * float(human::k_shoulder_width_adult_male);
      return !std::isfinite(calc_height) ? r : (r * calc_height / height());
   }

   // The centre of the detection -- the hist position is projected downards
   Vector3f Xs_centre() const noexcept;

   const Vector3f X(float calc_height = fNAN) const noexcept
   {
      return !std::isfinite(calc_height)
                 ? X_
                 : C_ + (calc_height / height()) * (X_ - C_);
   }
   const auto& floor_line() const noexcept { return floor_line_; }
   const auto& floor_ray() const noexcept // points away from cam
   {
      return floor_ray_;
   }

   float seconds() const noexcept { return seconds_; }
   const auto& Xs() const noexcept { return Xs_; }
   vector<Vector3f> calc_Xs(float calc_height = fNAN) const noexcept
   {
      if(!std::isfinite(calc_height)) return Xs();
      auto out          = Xs();
      const float ratio = calc_height / height();
      for(auto& X : out) X = C_ + ratio * (X - C_);
      return out;
   }

   float reproj_err() const noexcept { return reproj_err_; }

   // These functions return a number in [0..1], of fNAN
   float ankle_proportion() const noexcept;
   float knee_proportion() const noexcept;
   float pelvis_proportion() const noexcept;
   float notch_proportion() const noexcept;
   float nose_proportion() const noexcept;

   size_t memory_usage() const noexcept;
};

inline Skeleton3D fit_3d_ret(const DistortedCamera* dcam_ptr,
                             const Skeleton2D& p2d,
                             const bool feedback) noexcept(false)
{
   return Skeleton3D::fit_3d_ret(dcam_ptr, p2d, feedback);
}

void plot_skeleton_3d(const DistortedCamera& dcam,
                      const Skeleton3D& s3d,
                      const float height, // pass fNAN for default height
                      const AABB& im_bounds,
                      std::function<void(int, int, float)> f);

float shape_score(const Skeleton3D& s3d, const bool feedback = false) noexcept;

} // namespace perceive
