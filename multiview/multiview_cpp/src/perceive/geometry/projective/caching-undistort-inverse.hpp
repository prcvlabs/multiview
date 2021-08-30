
#pragma once

#include "polynomial-model.hpp"

namespace perceive
{
// -----------------------------------------------------------------------------
// --                        CachingUndistortInverse                          --
// -----------------------------------------------------------------------------

struct CachingUndistortInverse
{
 private:
   AABB aabb_;
   string A_str_{""}; // md5 of parameters used create field

   shared_ptr<EmpiricalFieldF> field_;
   Vector2 calib_fmt{0.0, 0.0};
   Vector2 scale_{1.0, 1.0}; // To rescale outputs (distorted points)
   DistortionModel M_;
   bool is_K_{false};
   Matrix3r K_;
   Matrix3r K_inv_;

   // We use approximations for
   struct ApproxK
   {
      Vector2 Dc_, Uc_;    // The centre (distorted/undistorted) of this approx.
      Matrix3r K_, K_inv_; // Rectiliniear approx. for distortion

      Vector2 distort(const Vector2& U) const noexcept;   // O(1)
      Vector2 undistort(const Vector2& D) const noexcept; // O(1)
   };
   vector<ApproxK> approx_Ks_; // Should use 8 approximations

   // Could return nullptr if approx_Ks_ is empty
   const ApproxK* nearest_to_undistorted(const Vector2& U) const;
   const ApproxK* nearest_to_distorted(const Vector2& D) const;

   Vector2 approx_distort(const Vector2& U) const noexcept;
   Vector2 approx_undistort(const Vector2& D) const noexcept;

   void init_approx_Ks_();

   void reinit(const DistortionModel& M,
               const AABBi aabb,
               const float pixel_size,
               const bool use_format,
               const string& key);

   // Returns the maximum error for distorting a selection of five points.
   static real distort_fidelity_check(const CachingUndistortInverse& cu,
                                      const bool feedback = false) noexcept;

 public:
   CUSTOM_NEW_DELETE(CachingUndistortInverse)

   CachingUndistortInverse()
       : field_(make_shared<EmpiricalFieldF>())
   {}

   CachingUndistortInverse(const DistortionModel& M,
                           const bool force_recalc = false,
                           // in distorted space
                           const AABBi aabb              = AABBi{0, 0, 0, 0},
                           const bool default_full_image = true,
                           const float pixel_size        = 1.0f,
                           const bool feedback           = false)
       : field_(make_shared<EmpiricalFieldF>())
   {
      init(M, force_recalc, aabb, default_full_image, pixel_size, feedback);
   }

   CachingUndistortInverse(const Matrix3r& K) { init(K); }

   CachingUndistortInverse(const CachingUndistortInverse&) = default;
   CachingUndistortInverse(CachingUndistortInverse&&)      = default;
   ~CachingUndistortInverse()                              = default;
   CachingUndistortInverse& operator=(const CachingUndistortInverse&) = default;
   CachingUndistortInverse& operator=(CachingUndistortInverse&&) = default;

   void init(AABB aabb, const DistortionModel& M, EmpiricalFieldF&& field);

   void init(const Matrix3r& K);

   void init(const DistortionModel& M,
             const bool force_recalc = false,
             // in distorted space
             const AABBi aabb              = AABBi{0, 0, 0, 0},
             const bool default_full_image = true,
             const float pixel_size        = 1.0f,
             const bool feedback           = false);

   const auto& aabb() const noexcept { return aabb_; }
   const auto& field() const noexcept { return *field_; }
   const auto& A_str() const noexcept { return A_str_; }
   const auto& scale() const noexcept { return scale_; }
   const auto& M() const noexcept { return M_; }

   // Scales the distorted output values
   void set_working_format(const real distorted_w,
                           const real distorted_h) noexcept
   {
      scale_ = Vector2(distorted_w / calib_fmt.x, distorted_h / calib_fmt.y);
      M_.set_working_format(unsigned(distorted_w), unsigned(distorted_h));
   }

   Vector2 working_format() const noexcept { return M_.working_format(); }

   bool in_bounds(const Vector2f& x) const noexcept
   {
      return is_K_ ? true : aabb_.contains(real(x.x), real(x.y));
   }
   bool in_bounds(const Vector2& x) const noexcept
   {
      return is_K_ ? true : aabb_.contains(x.x, x.y);
   }

   Vector2 undistort(const Vector2& x) const noexcept
   {
      if(is_K_)
         return homgen_P2_to_R2(to_vec3(K_inv_ * Vector3r(x.x, x.y, 1.0)));
      return M_.undistort(x);
   }

   Vector2f undistort(const Vector2f& x) const noexcept
   {
      return to_vec2f(undistort(to_vec2(x)));
   }

   Vector2f distort(const Vector2f& x) const noexcept
   {
      if(is_K_) return to_vec2f(distort(to_vec2(x)));

      auto f = field_->evaluate(x);
      if(f.is_finite())
         return Vector2f(f.x * float(scale_.x), f.y* float(scale_.y));
      const Vector3 y = M_.distort(to_vec3(Vector3f(x.x, x.y, 1.0)));
      return to_vec2f(Vector2(y.x, y.y));
   }

   Vector2 distort(const Vector2& x) const noexcept
   {
      assert(x.is_finite());
      if(is_K_) return homgen_P2_to_R2(to_vec3(K_ * Vector3r(x.x, x.y, 1.0)));
      auto f = M_.distort(Vector2(x.x, x.y));
      return Vector2(f.x, f.y);
   }

   Vector2r distort(const Vector2r& x) const noexcept
   {
      if(is_K_) return to_vec2r(distort(to_vec2(x)));

      auto f = distort(Vector2(x(0), x(1)));
      return Vector2r(f.x, f.y);
   }

   Vector3r distort(const Vector3r& x) const noexcept
   {
      if(is_K_) return K_ * x;

      auto y = normalized_P2(x);
      auto f = distort(to_vec2f(Vector2(y(0), y(1))));
      return Vector3r(real(f.x), real(f.y), 1.0);
   }

   Vector2f fast_distort(const Vector2f& x) const noexcept
   {
      if(is_K_) return to_vec2f(distort(to_vec2(x)));
      auto f = field_->evaluate(x);
      return Vector2f(f.x * float(scale_.x), f.y* float(scale_.y));
   }

   Vector2 fast_distort(const Vector2& x) const noexcept
   {
      if(is_K_) return homgen_P2_to_R2(to_vec3(K_ * Vector3r(x.x, x.y, 1.0)));
      return to_vec2(fast_distort(to_vec2f(x)));
   }

   Vector2f slow_distort(const Vector2f& x) const noexcept
   {
      if(is_K_) return to_vec2f(distort(to_vec2(x)));
      auto y = M_.distort(Vector3(real(x.x), real(x.y), 1.0));
      return Vector2f(float(y.x), float(y.y));
   }

   Vector2 slow_distort(const Vector2& x) const noexcept
   {
      if(is_K_) return homgen_P2_to_R2(to_vec3(K_ * Vector3r(x.x, x.y, 1.0)));
      auto y = M_.distort(Vector3(x.x, x.y, 1.0));
      return Vector2(y.x, y.y);
   }

   friend void load(CachingUndistortInverse& data,
                    const string& fname) noexcept(false);
   friend void save(const CachingUndistortInverse& data,
                    const string& fname) noexcept(false);
   friend void read(CachingUndistortInverse& data, FILE* fp) noexcept(false);
   friend void write(const CachingUndistortInverse& data,
                     FILE* fp) noexcept(false);

   friend void load_xdr(CachingUndistortInverse& data,
                        const string& fname) noexcept(false);
   friend void save_xdr(const CachingUndistortInverse& data,
                        const string& fname) noexcept(false);
   friend void read_xdr(CachingUndistortInverse& data,
                        FILE* fp) noexcept(false);
   friend void write_xdr(const CachingUndistortInverse& data,
                         FILE* fp) noexcept(false);
};

} // namespace perceive
