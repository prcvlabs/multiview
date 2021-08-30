
#include "gl-z-buffer.hpp"

#include <GL/gl.h>

#define This GlZBuffer

namespace perceive
{
// ------------------------------------------------------------------ Grey Image

GreyImage This::to_grey_image() const
{
   GreyImage g;
   g.resize(z_buffer.width, z_buffer.height, z_buffer.width);

   auto [min_val, max_val] = z_buffer.minmax();
   float range_inv         = 1.0f / (max_val - min_val);

   auto src = z_buffer.cbegin();
   auto end = z_buffer.cend();
   auto dst = g.begin();
   while(src != end)
      *dst++ = GreyImage::value_type(255.0f * range_inv * (*src++ - min_val));

   return g;
}

// --------------------------------------------------------------- Read From FBO

void This::read_from_fbo(const GlProjection& port, const FramebufferObject& fbo)
{
   const unsigned fbo_w = fbo.w();
   const unsigned fbo_h = fbo.h();

   // Initialize the z-buffer
   if(z_buffer.width != fbo_w or z_buffer.height != fbo_h)
      z_buffer.resize(fbo_w, fbo_h, fbo_w);

   // Let's read the depth data
   fbo.bind();
   glReadPixels(0,
                0,
                GLsizei(fbo_w),
                GLsizei(fbo_h),
                GL_DEPTH_COMPONENT,
                GL_FLOAT,
                z_buffer.data());

   {
      auto end = z_buffer.end();
      for(auto ii = z_buffer.begin(); end != ii; ++ii) {
         // cout << format("{} => {}", *ii, port.convert_z(*ii)) << endl;
         *ii = float(port.convert_z(real(*ii)));
      }
   }

   return;
   z_buffer.vertical_flip();
}

// ------------------------------------------------------------ Tangent Plane At
// Returns a tange plane estimaged at point 'X'
Plane tangent_plane_at(const GlProjection& port,
                       const GlZBuffer& z_buffer,
                       const Vector2& X)
{
   const auto A = X + Vector2{2.0, 2.0};
   const auto B = X + Vector2{-2.0, 2.0};
   const auto C = X + Vector2{-2.0, -2.0};
   return Plane(z_buffer.screen_to_world(port, A),
                z_buffer.screen_to_world(port, B),
                z_buffer.screen_to_world(port, C));
}

} // namespace perceive
