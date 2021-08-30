
#pragma once

#include "optimize-camera-position.hpp"
#include "perceive/geometry.hpp"
#include "perceive/graphics.hpp"

#ifdef USING_OPENGL
#include "gl/gl.hpp"
#endif

namespace perceive
{
struct CameraPosOptContext
{
   CameraPosOptContext() = delete;
   CameraPosOptContext(const OptimizeCameraData& o);
   CameraPosOptContext(const CameraPosOptContext&) = delete;
   CameraPosOptContext(CameraPosOptContext&&)      = default;
   ~CameraPosOptContext()                          = default;
   CameraPosOptContext& operator=(const CameraPosOptContext&) = delete;
   CameraPosOptContext& operator=(CameraPosOptContext&&) = default;

   void init(const unsigned cam_ind, const bool feedback = false);

   const OptimizeCameraData* data = nullptr;
   bool feedback{false};

#ifdef USING_OPENGL
   unique_ptr<FramebufferObject> fbo_ptr;
   GlProjection port;
   GlZBuffer z_buffer;
#endif

   ARGBImage argb_buffer;

   unsigned camera_index{0}; // The index of the current camera
   string sensor_id() const noexcept
   {
      Expects(data);
      return data->scene_desc.sensor_ids[camera_index];
   }
   DistortionModel model;
   CachingUndistortInverse distort_f;

   // Displaylist for CAD model (not owned)
   unsigned display_list_index{0};

   // Data
   cv::Mat grey;
   BinaryImage well_calibrated;
   ImageFeatures2d f2d;

   // Helper functions
   const Vector3& cam_center() const noexcept
   {
#ifndef USING_OPENGL
      WARN("opengl not compiled in");
      static Vector3 C;
      return C;
#else
      return port.cam_et().translation;
#endif
   }

   // OpenGL screen-pixel to world coordinate
   Vector3 screen_to_world(const Vector2& s) const noexcept
   {
#ifndef USING_OPENGL
      WARN("opengl not compiled in");
      static Vector3 C;
      return C;
#else
      return z_buffer.screen_to_world(port, s);
#endif
   }

   // Regular (image) pixel to world coordinate
   Vector3 regular_screen_to_world(const Vector2& s) const noexcept
   {
#ifndef USING_OPENGL
      WARN("opengl not compiled in");
      static Vector3 C;
      return C;
#else
      return z_buffer.regular_screen_to_world(port, s);
#endif
   }

   // Pixel in distorted image... to world-coordinate, via z-buffer
   Vector3 distorted_to_world(const Vector2& D) const noexcept
   {
#ifndef USING_OPENGL
      WARN("opengl not compiled in");
      static Vector3 C;
      return C;
#else
      const auto U = model.undistort(D);
      const auto R = Vector3(U(0), U(1), 1.0);
      return z_buffer.screen_to_world(port, port.regular_eye_to_screen(R));
#endif
   }

   Vector2 world_to_distorted(const Vector3& W) const noexcept
   {
#ifndef USING_OPENGL
      WARN("opengl not compiled in");
      static Vector2 C;
      return C;
#else
      const auto E = port.cam_et().inverse_apply(W);
      return distort_f.distort(Vector2(E(0) / E(2), E(1) / E(2)));
#endif
   }

   void calculate_depth_in_fbo() const; // NOTE: does not read result
   void read_depth_result_from_fbo();

   void debug_export_gl_result(const string& base_directory);
};

// calculates depths in FBO, and then (multi-threaded) reads the result
void update_depths(const vector<unsigned>& cam_inds, // cams of interest
                   vector<CameraPosOptContext>& opt_ctxs,
                   const bool export_files = false);

} // namespace perceive
