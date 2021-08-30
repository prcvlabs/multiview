
#include "camera-pos-opt-context.hpp"

#ifdef USING_OPENGL
#include <GL/gl.h>
#endif

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/file-system.hpp"
#include "photo-consistency.hpp"

#define This CameraPosOptContext

namespace perceive
{
// ---------------------------------------------------------------- Construction

This::This(const OptimizeCameraData& o)
    : data(&o)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   Expects(data && data->scene_desc.cad_model_ptr);

   // Create the frame-buffer object
   fbo_ptr = make_unique<FramebufferObject>(data->fbo_w, data->fbo_h);
#endif
}

// ------------------------------------------------------------------------ Init

void This::init(const unsigned sensor_ind, const bool feedback_)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   Expects(data && sensor_ind < data->n_sensors());

   camera_index = sensor_ind;
   feedback     = feedback_;

   // Set up the port
   port.init(0.0,
             0.0,
             data->fbo_w,
             data->fbo_h,
             data->hfov,
             data->z_near,
             data->z_far);
   port.set_cam_et_from_perceive_cam_et(
       data->scene_desc.sensor_transforms[camera_index]);

   // Fetch the model and distort-inverse
   const auto bcam_sensor_ind = data->scene_desc.bcam_lookup(int(camera_index));
   model = data->scene_desc.bcam_infos[size_t(bcam_sensor_ind.x)]
               .M[size_t(bcam_sensor_ind.y)];
   distort_f.init(model);

   // Create 2d features
   const auto& raw        = data->scene_desc.sensor_image[camera_index];
   const auto& f2d_params = data->f2d_params[camera_index];
   cv::cvtColor(raw, grey, cv::COLOR_BGR2GRAY);
   init_2d_features(
       f2d, raw, grey, nullptr, f2d_params, [&]() { return false; });

   make_slic_labeled_image(f2d, false)
       .save(format("/tmp/z_{:2d}_slic.png", sensor_ind));

   const auto& hull = model.calib_hull();
   well_calibrated.resize(grey.cols, grey.rows);
   well_calibrated.fill(false);
   for(auto y = 0u; y < well_calibrated.height; ++y)
      for(auto x = 0u; x < well_calibrated.width; ++x)
         well_calibrated(x, y) = model.point_in_calib_region(Vector2(x, y));
   well_calibrated.save("/tmp/well-calibrated.png");
#endif
}

// ------------------------------------------------------ Debug Export Gl Result

void This::debug_export_gl_result(const string& base_directory)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   const auto& port = this->port;
   const auto fbo_w = this->port.w;
   const auto fbo_h = this->port.h;

   mkdir_p(base_directory);

   z_buffer.read_from_fbo(port, *fbo_ptr);
   fbo_ptr->bind();

   auto fname = [&](string m) {
      return format("{}/z_{:2d}_{}", base_directory, camera_index, m);
   };

   GreyImage z_im = z_buffer.to_grey_image();
   z_im.save(fname("zbuffer.png"));

   // Let's read the color data
   auto& argb_buffer0 = argb_buffer;
   argb_buffer0.resize(int(fbo_w), int(fbo_h));
   glReadBuffer(GL_COLOR_ATTACHMENT0);
   glReadPixels(0,
                0,
                GLsizei(fbo_w),
                GLsizei(fbo_h),
                GL_BGRA,
                GL_UNSIGNED_BYTE,
                argb_buffer0.data());
   render_string(argb_buffer0, sensor_id(), Point2(10, 16), k_yellow, k_black);

   argb_buffer0.save(fname("rendered.png"));

   ARGBImage raw;
   cv_to_argb(data->scene_desc.sensor_image[camera_index], raw);
   raw.save(fname("raw.png"));
   const auto o_w = raw.width;
   const auto o_h = raw.height;

   ARGBImage argb;
   argb.resize(int(fbo_w), int(fbo_h));
   distort_f.set_working_format(o_w, o_h);

   for(auto y = 0u; y < fbo_h; ++y) {
      for(auto x = 0u; x < fbo_w; ++x) {
         argb(x, y) = grey_to_rgb(255 - z_im(x, y));

         // Get the ray...
         auto n = to_R2_coord(port, Vector2(x, y));

         // Back project to distorted image
         auto p = to_pt2(distort_f.distort(n).round());

         // If in bounds, blend the color
         if(raw.in_bounds(p)) argb(x, y) = blend(argb(x, y), raw(p), 0.75);
      }
   }
   argb.save(fname("blended.png"));
#endif
}

// ------------------------------------------------------ calculate-depth-in-fbo
// NOTE: does not read result
void This::calculate_depth_in_fbo() const
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   const auto& cad_model = *data->scene_desc.cad_model_ptr;

   // Bind the off-screen (frame) buffer
   // cout << format("binding fbo {}, cam-ind = {}",
   //                fbo, ctx.camera_index) << endl;
   check_and_warn_on_gl_error("before binding FBO");
   fbo_ptr->bind();

   // Clear everything that was there
   glClearColor(0.0, 0.0, 0.0, 0.0);
   glClearDepth(1.0);
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

   // Set up the viewport, and projection matrix
   port.gl_projection_apply();
   port.gl_modelview_apply();

   // Now we render
   if(display_list_index > 0)
      glCallList(display_list_index);
   else
      render_gl(cad_model);
   check_and_warn_on_gl_error("after rendering CAD model");
#endif
}

// -------------------------------------------------- read-depth-result-from-fbo

void This::read_depth_result_from_fbo()
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   z_buffer.read_from_fbo(port, *fbo_ptr);
#endif
}

// --------------------------------------------------------------- update-depths

void update_depths(const vector<unsigned>& sensor_inds, // cams of interest
                   vector<CameraPosOptContext>& opt_ctxs,
                   const bool export_files)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   auto read_i = [&](unsigned i) {
      auto& ctx = opt_ctxs[i];
      ctx.read_depth_result_from_fbo();
      if(export_files) ctx.debug_export_gl_result("/tmp");
   };

   for(auto i : sensor_inds) opt_ctxs[i].calculate_depth_in_fbo();
   glFinish();

   // This is actually the slow part...
   ParallelJobSet jobs;
   for(auto i : sensor_inds) jobs.schedule([i, read_i]() { read_i(i); });
   jobs.execute();
#endif
}

} // namespace perceive
