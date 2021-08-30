
#include "blend-im-with-CAD.hpp"
#include "gl-inc.hpp"

#include "perceive/graphics/colour-set.hpp"

#include <GL/gl.h>

namespace perceive
{
ARGBImage blend_im_with_CAD(const ARGBImage& raw,
                            const DistortionModel& M,
                            const EuclideanTransform& et_opt,
                            const Sprite& cad,
                            const int fbo_w,
                            const int fbo_h,
                            const real fbo_hfov)
{
   GlProjection port;
   GlZBuffer z_buffer;

   ensure_glew_and_gl_setup();
   stock_gl_setup(unsigned(fbo_w), unsigned(fbo_h));

   const auto z_far = (cad.min_xyz() - cad.max_xyz()).norm();
   if(!std::isfinite(z_far))
      FATAL(format("z-far wasn't finite, CAD.min = {:s}, CAD.max = {:s}",
                   str(cad.min_xyz()),
                   str(cad.max_xyz())));

   check_and_warn_on_gl_error("before making FBO");
   auto fbo_ptr = make_unique<FramebufferObject>(fbo_w, fbo_h);
   check_and_warn_on_gl_error("after making FBO");

   check_and_warn_on_gl_error("before binding FBO");
   fbo_ptr->bind();
   check_and_warn_on_gl_error("after binding FBO");

   // Clear everything that was there
   glClearColor(0.0, 0.0, 0.0, 0.0);
   glClearDepth(1.0);
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

   // Set up the viewport, and projection matrix
   port.init(0.0, 0.0, fbo_w, fbo_h, fbo_hfov, 0.100, z_far);
   port.set_cam_et_from_perceive_cam_et(et_opt);
   port.gl_projection_apply();
   port.gl_modelview_apply();

   // INFO("-------------");
   // cout << "et = " << str(et_opt) << endl;
   // cout << "P  = " << endl << port.projection_matrix() << endl << endl;
   // cout << "MV = " << endl << port.modelview_matrix() << endl << endl;

   // Now render
   check_and_warn_on_gl_error("before rendering CAD model");
   render_gl(cad);
   check_and_warn_on_gl_error("after rendering CAD model");
   glFinish();

   z_buffer.read_from_fbo(port, *fbo_ptr);

   GreyImage z_im = z_buffer.to_grey_image();

   ARGBImage argb;
   argb.resize(fbo_w, fbo_h);

   CachingUndistortInverse cu;
   cu.init(M);
   cu.set_working_format(raw.width, raw.height);

   for(auto y = 0; y < fbo_h; ++y) {
      for(auto x = 0; x < fbo_w; ++x) {
         argb(x, y) = grey_to_rgb(255 - z_im(x, y));

         // Get the ray...
         auto n = to_R2_coord(port, Vector2(x, y));

         // Back project to distorted image
         auto p = to_pt2(cu.distort(n).round());

         // If in bounds, blend the color
         if(raw.in_bounds(p)) argb(x, y) = blend(argb(x, y), raw(p), 0.75);
      }
   }

   return argb;
}

} // namespace perceive
