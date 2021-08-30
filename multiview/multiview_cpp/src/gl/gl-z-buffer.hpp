
#pragma once

#include "framebuffer-object.hpp"
#include "gl-projection.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
// A wrapper around the OpenGL z-buffer object
struct GlZBuffer
{
 public:
   FloatImage z_buffer; // Contains the NDC.z values of Gl "depth" buffer

   GreyImage to_grey_image() const;

   unsigned w() const noexcept { return z_buffer.width; }
   unsigned h() const noexcept { return z_buffer.height; }

   // Initialize the z-buffer from OpenGL
   // NOTE: flips the y-coord of OpenGL
   void read_from_fbo(const GlProjection& port, const FramebufferObject& fbo);

   // Reading values out of the z-buffer
   real read_z(Point2 x) const noexcept
   {
      return z_buffer.in_bounds(x) ? real(z_buffer(x)) : dNAN;
   }
   real read_z(const Vector2f& x) const noexcept
   {
      return read_z(Point2(int(std::round(x.x)), int(std::round(x.y))));
   }
   real read_z(const Vector2& x) const noexcept
   {
      return read_z(Point2(int(std::round(x.x)), int(std::round(x.y))));
   }

   // Screen to...
   Vector3 screen_to_eye(const GlProjection& port,
                         const Vector2& s) const noexcept
   {
      return port.screen_to_eye(s, read_z(s));
   }
   Vector3 screen_to_world(const GlProjection& port,
                           const Vector2& s) const noexcept
   {
      return port.screen_to_world(s, read_z(s));
   }

   Vector3 regular_screen_to_eye(const GlProjection& port,
                                 const Vector2& s) const noexcept
   {
      return port.regular_screen_to_eye(s, read_z(s));
   }
   Vector3 regular_screen_to_world(const GlProjection& port,
                                   const Vector2& s) const noexcept
   {
      return port.eye_to_world(regular_screen_to_eye(port, s));
   }
};

// Returns a tange plane estimated at point 'X'
Plane tangent_plane_at(const GlProjection& port,
                       const GlZBuffer& z_buffer,
                       const Vector2& X);

} // namespace perceive
