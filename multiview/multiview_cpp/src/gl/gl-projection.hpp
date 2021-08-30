
#pragma once

#include "gl-utils.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
struct GlProjection
{
   GlProjection(real x_ = 0.0, real y_ = 0.0, real w_ = 640.0, real h_ = 480.0,
                real hfov_radians_ = to_radians(90.0), real z_near_ = 0.01,
                real z_far_ = 100.0)
   {
      init(x_, y_, w_, h_, hfov_radians_, z_near_, z_far_);
   }
   GlProjection(const GlProjection&) = default;
   GlProjection(GlProjection&&)      = default;
   ~GlProjection()                   = default;

   GlProjection& operator=(const GlProjection&) = default;
   GlProjection& operator=(GlProjection&&) = default;

   void init(real x_, real y_, real w_, real h_, real hfov_radians_,
             real z_near_, real z_far_) noexcept;

   // Viewport
   real x{0.0}; // Specify the lower left corner of the viewport, in pixels
   real y{0.0};
   real w{0.0}; // Specify the width and height of the viewpory
   real h{0.0};

   // Projection
   real hfov{0.0};
   real z_near{0.0};
   real z_far{0.0};

   static const EuclideanTransform et0;

   real focal_length() const noexcept { return h / (2.0 * tan(hfov * 0.5)); }

 private:
   // Derived terms
   real aspect{0.0};     // w/h
   real aspect_inv{0.0}; // h/w
   real f{0.0};          // atan(hfov * 0.5)
   real f_inv{0.0};      // 1/f
   real z_n_plus_f{0.0}; // z_near + n_far
   real z_n_m_f{0.0};    // z_near - z_far
   real z_2nf{0.0};      // 2.0 * z_near * z_far

   // Camera location/orientation
   EuclideanTransform gl_et_;
   EuclideanTransform cam_et_;

 public:
   // What was used to "set-cam-et-from-perceive-cam-et"
   const EuclideanTransform& cam_et() const noexcept { return cam_et_; }
   // OpenGL looks down the -ve Z axis, so this corrects cam_et as such
   const EuclideanTransform& gl_et() const noexcept { return gl_et_; }

   void debug_world_to_screen(const Vector3& W) const noexcept;

   // Note that OpenGL looks down -Z, but we want to look down +Z
   void set_cam_et_from_perceive_cam_et(const EuclideanTransform& et)
   {
      // Start by pointing DOWN the +Z axis, with Y up, and X right
      cam_et_ = et;
      gl_et_  = et.inverse() * et0;
   }

   // @{
   // World -->
   Vector3 world_to_eye(const Vector3& X) const noexcept
   {
      return gl_et_.apply(X);
   }

   Vector3 world_to_ray(const Vector3& X) const noexcept
   {
      return world_to_eye(X).normalized();
   }

   Vector4 world_to_clip(const Vector3& X) const noexcept
   {
      return eye_to_clip(world_to_eye(X));
   }

   Vector3 world_to_NDC(const Vector3& X) const noexcept
   {
      return eye_to_NDC(world_to_eye(X));
   }

   Vector2 world_to_screen(const Vector3& X) const noexcept
   {
      return eye_to_screen(world_to_eye(X));
   }

   // Eye -->
   Vector3 eye_to_world(const Vector3& X) const noexcept
   {
      return gl_et_.inverse_apply(X);
   }

   Vector4 eye_to_clip(const Vector3& X) const noexcept
   {
      const auto z_inv = -1.0 / X.z;
      Vector4 C;
      C.x = f * aspect_inv * X.x;
      C.y = f * X.y;
      C.z = (X.z * z_n_plus_f + z_2nf) / z_n_m_f;
      C.w = -X.z;
      return C;
   }

   Vector3 eye_to_NDC(const Vector3& X) const noexcept
   {
      auto C = eye_to_clip(X);
      C /= C(3);
      return Vector3{C(0), C(1), C(2)};
   }

   Vector2 eye_to_screen(const Vector3& X) const noexcept
   {
      return NDC_to_screen(eye_to_NDC(X));
   }
   Vector2 ray_to_screen(const Vector3& X) const noexcept
   {
      return eye_to_screen(X);
   }

   // NDC -->
   Vector2 NDC_to_screen(const Vector3& NDC) const noexcept
   {
      return Vector2{(NDC.x + 1.0) * 0.5 * w + x, (NDC.y + 1.0) * 0.5 * h + y};
   }

   // Screen -->
   Vector2 screen_to_NDC(const Vector2& X) const noexcept
   {
      auto nx = 2.0 * (X.x - x) / w - 1.0;
      auto ny = 2.0 * (X.y - y) / h - 1.0;
      return Vector2(nx, ny);
   }

   Vector3 screen_to_eye(const Vector2& X, real z) const noexcept
   {
      const auto n = screen_to_NDC(X);
      return Vector3{-z * aspect * n.x * f_inv, -z * n.y * f_inv, z};
   }

   Vector3 screen_to_ray(const Vector2& X) const noexcept
   {
      const auto r = screen_to_normalized_coord(X);
      return Vector3{-r.x, -r.y, -1.0}.normalized();
   }

   Vector2 screen_to_normalized_coord(const Vector2& X) const noexcept
   {
      const auto n = screen_to_NDC(X);
      return Vector2{-aspect * n.x * f_inv, -n.y * f_inv};
   }

   Vector3 screen_to_world(const Vector2& X, real z) const noexcept
   {
      return eye_to_world(screen_to_eye(X, z));
   }
   // @}

   // @{
   // The OpenGL coord system has the y-axis flipped.
   // These functions take those flipped coords into account,
   // allowing for recovery
   Vector2 flip_y(const Vector2& X) const noexcept
   {
      return Vector2(X.x, h - X.y);
   }

   Vector2 regular_screen_to_normalized_coord(const Vector2& X) const noexcept
   {
      return -screen_to_normalized_coord(flip_y(X));
   }

   Vector3 regular_screen_to_eye(const Vector2& X, real z) const noexcept
   {
      const auto n = regular_screen_to_normalized_coord(X);
      return Vector3{-z * n.x, -z * n.y, z};
   }

   Vector2 eye_to_regular_screen(const Vector3& X) const noexcept
   {
      return eye_to_screen(Vector3(X(0), -X(1), X(2)));
   }

   Vector2 regular_eye_to_screen(const Vector3& X) const noexcept
   {
      return eye_to_screen(Vector3(X(0), -X(1), X(2)));
   }
   // @}

   // @{
   // This value must be read from the depth buffer
   // NOTE: this function returns a -ve value, because
   // OpenGL looks down the -ve Z axis
   real convert_z(const real depth_buffer_z) const noexcept
   {
      // wz is the NDC z value
      const real z = (2.0 * depth_buffer_z) - 1.0; // [0..1] -> [-1..1]
      return -z_2nf / (z * z_n_m_f + z_n_plus_f);
   }
   // @}

   // @{
   // Create a projection matrix (WARNING row-major order)
   Matrix4r projection_matrix() const noexcept;

   Matrix4r modelview_matrix() const noexcept
   {
      return make_transform_matrix(gl_et_);
   }
   // @}

   // @{
   void reshape(int width, int height) noexcept
   {
      Expects(width > 0);
      Expects(height > 0);
      init(x, y, width, height, hfov, z_near, z_far);
   }

   void reshape_and_apply(int width, int height) noexcept
   {
      reshape(width, height);
      gl_projection_apply();
   }

   // calls glViewport and gluPerspective
   void gl_projection_apply() const noexcept;
   void gl_modelview_apply() const noexcept;
   //@}
};

// Returns a regular P2 homgen coordinate, with the Z-plane at
// Z=1.0, and also taking care to flip the y-axis when interpreting
// S. Boo OpenGL!
inline auto to_R2_coord(const GlProjection& p, const Vector2& S) noexcept
{
   return -p.regular_screen_to_normalized_coord(S);
}

} // namespace perceive
