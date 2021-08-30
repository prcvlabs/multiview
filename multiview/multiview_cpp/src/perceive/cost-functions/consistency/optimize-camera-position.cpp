
#ifdef USING_OPENGL
#include <GL/glew.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include "gl/gl-inc.hpp"
#include "gl/gl.hpp"
#endif

#include "camera-pos-opt-context.hpp"
#include "optimize-camera-position.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/file-system.hpp"
#include "photo-consistency.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

namespace perceive
{
// ------------------------------------------------------------ Static Variables

static ParallelJobSet jobs;

// STR00003, full sized image
static vector<unsigned> slic_str3_interest{{
    184, // table-edge
    572, // table-center
    971, // floor
    765, // floor-edge
}};

static Vector2 transfer_left_right(const BinocularCameraInfo& bcam,
                                   const CachingUndistortInverse& cu,
                                   const EuclideanTransform& et,
                                   const Plane& p3,
                                   const Vector2& D)
{
   const auto& C = et.translation;
   const auto R  = bcam.M[0].undistort(Vector3(D.x, D.y, 1.0));
   const auto Rr = et.rotation.rotate(R);
   const auto X1 = plane_ray_intersection(p3, C, C + Rr);
   const auto E1 = et.rotation.inverse_rotate(X1 - C);
   const auto C1 = -bcam.q.inverse_rotate(bcam.t.normalised() * bcam.baseline);
   const auto U2 = homgen_P2_to_R2(bcam.q.rotate(E1 - C1));
   const auto y  = cu.in_bounds(U2) ? cu.distort(U2) : Vector2::nan();
   // bcam.M[1].distort(U2);

   if(false) {
      cout << format("M .fmt = {}", str(bcam.M[0].working_format())) << endl;
      cout << format("X1 = {}", str(X1)) << endl;
      cout << format("E1 = {}", str(E1)) << endl;
      cout << format("C1 = {}", str(C1)) << endl;
      cout << format("U2 = {}", str(U2)) << endl;
      cout << format("y  = {}", str(y)) << endl;
   }

   return y;
}

// ------------------------------------------------------------------ Initialize

static auto init_optimize_operation(const OptimizeCameraData& data,
                                    const bool feedback_)
{
   Expects(data.n_sensors() > 0);
   vector<CameraPosOptContext> opt_ctxs;

#ifndef USING_OPENGL
   FATAL("Operation will not work at all: no OpenGl");
#else
   stock_gl_setup(data.fbo_w, data.fbo_h);
   INFO(format("frame-buffer wh is [{}, {}]", data.fbo_w, data.fbo_h));

   // Set up the contexts
   for(auto i = 0u; i < data.n_sensors(); ++i) {
      cout << format("Loading sensor: {}", data.scene_desc.sensor_ids[i])
           << endl;
      opt_ctxs.emplace_back(data);
   }

   {
      ParallelJobSet jobs;
      auto process_i = [&](auto i) { opt_ctxs[i].init(i, feedback_); };
      for(auto i = 0u; i < data.n_sensors(); ++i)
         jobs.schedule([i, process_i]() { process_i(i); });
      jobs.execute_non_parallel();
   }

   if(false) {
      GLint depth_bits{0};
      glGetIntegerv(GL_DEPTH_BITS, &depth_bits);
      WARN(format("depth-bits = {}", depth_bits));
   }
#endif
   return opt_ctxs;
}

// --------------------------------------------------------------- Update Depths

static void update_depths(vector<CameraPosOptContext>& opt_ctxs)
{
   vector<unsigned> cam_inds(opt_ctxs.size());
   std::iota(begin(cam_inds), end(cam_inds), 0);
   update_depths(cam_inds, opt_ctxs, false);
}

// ------------------------------------------------------------

static real optimize_camera_pair(vector<unsigned> cam_inds,
                                 vector<CameraPosOptContext>& opt_ctxs)
{
   return 0.0;
}

// ------------------------------------------- Calc Superpixel Photo Consistency
// Returns [0..1]
static real
score_spixel_consistency(const unsigned cam_ind,
                         const unsigned spixel_ind,
                         const Plane& label,
                         const vector<CameraPosOptContext>& opt_ctxs)
{
   return 0.0;
}

// ------------------------------------------------------------ Label from model

static Plane label_from_depth(const CameraPosOptContext& ctx,
                              const unsigned spixel_ind)
{
#ifndef USING_OPENGL
   return Plane{};
#else
   Expects(spixel_ind < ctx.f2d.slic_numlabels);
   const auto& D = ctx.f2d.slic_info[spixel_ind].center;

   // Ray
   const auto U = ctx.model.undistort(D);
   const auto R = Vector3(U(0), U(1), 1.0);
   return tangent_plane_at(
       ctx.port, ctx.z_buffer, ctx.port.regular_eye_to_screen(R));
#endif
}

// ------------------------------------------------------------------ Score ctxs

static real score_ctxs(const vector<CameraPosOptContext>& opt_ctxs)
{
   // Calculate the photo-consistency for every super-pixel
   // Return the sum of all super-pixels that make the threshold
   auto score = 0.0;
   for(auto cam_index = 0u; cam_index < opt_ctxs.size(); ++cam_index) {
      const auto& opt_ctx = opt_ctxs[cam_index];
      auto n_labels       = opt_ctx.f2d.slic_numlabels;
      for(auto spixel_ind = 0u; spixel_ind < n_labels; ++spixel_ind) {
         auto label = label_from_depth(opt_ctx, spixel_ind);
         score += score_spixel_consistency(
             cam_index, spixel_ind, label, opt_ctxs);
      }
   }

   return 0.0;
}
// --------------------------------------------------------------------- Display

static vector<CameraPosOptContext> opt_ctxss;

static void display_function()
{
   const auto count = 1;
   auto s           = time_thunk([&]() {
      for(auto j = 0; j < count; ++j) update_depths(opt_ctxss);
   });

   cout << format("time = {}s", s / real(count * opt_ctxss.size())) << endl;

   // Export the images
   for(auto i = 0u; i < opt_ctxss.size(); ++i) {
      opt_ctxss[i].read_depth_result_from_fbo();
      opt_ctxss[i].debug_export_gl_result("/tmp/cam-pos/initial");
   }

   // prepare();
   // final();
   // glutSwapBuffers();
}

// ---------------------------------------------------------------------- Resize

static void resize_function(int width, int height)
{
   // if(data_ptr != nullptr and not first_resize and width != fbo_w)
   //     WARN(format("Please do NOT resize the window {}x{}", width,
   //     height));
   // first_resize = false;
}

// ---------------------------------------------------------------- to-lab-image

static Vec3fImage to_lab_im(const cv::Mat& im)
{
   ARGBImage argb;
   Vec3fImage o;

   cv_to_argb(im, argb);
   o.resize(im.cols, im.rows);

   for(auto y = 0u; y < o.height; ++y)
      for(auto x = 0u; x < o.width; ++x)
         o(x, y) = to_vec3f(rgb_to_lab(argb(x, y)));

   return o;
}

// ------------------------------------------------------- optimize-one-bino-cam

static void optimize_one_bino_cam(const OptimizeCameraData& data,
                                  const int bcam_ind,
                                  vector<CameraPosOptContext>& opt_ctxs)
{
#ifndef USING_OPENGL
   FATAL("opengl not compiled in");
#else
   const auto& scene          = data.scene_desc;
   const auto n_sensors       = data.n_sensors();
   const auto& bcam           = scene.bcam_infos[size_t(bcam_ind)];
   const auto s0_ind          = scene.sensor_lookup(bcam_ind, 0);
   const auto s1_ind          = scene.sensor_lookup(bcam_ind, 1);
   const auto lab0            = to_lab_im(scene.sensor_image[size_t(s0_ind)]);
   const auto lab1            = to_lab_im(scene.sensor_image[size_t(s1_ind)]);
   const auto n_params        = 6;
   const auto use_nelder_mead = true;
   const auto feedback        = true;

   CachingUndistortInverse cu;
   cu.init(bcam.M[1]);

   vector<unsigned> sensor_inds(1);
   sensor_inds[0] = unsigned(s0_ind);

   INFO(format("bcam[{}] = {}", bcam_ind, bcam.camera_id));
   cout << " + " << bcam.M[0].sensor_id() << endl;
   cout << " + " << bcam.M[1].sensor_id() << endl;

   const auto et0 = scene.aruco_transforms[size_t(s0_ind)];
   cout << str(et0) << endl << endl;
   EuclideanTransform et = et0;

   // ---- pack ----
   auto pack = [&](real* params) {
      Vector3 saa = quaternion_to_saa(et.rotation);
      for(auto i = 0; i < 3; ++i) *params++ = et.translation(i);
      for(auto i = 0; i < 3; ++i) *params++ = saa(i);
   };

   // ---- unpack ----
   auto unpack = [&](const real* params) {
      Vector3 saa;
      for(auto i = 0; i < 3; ++i) et.translation(i) = *params++;
      for(auto i = 0; i < 3; ++i) saa(i) = *params++;
      et.rotation = saa_to_quaternion(saa);
      et.scale    = 1.0;
      opt_ctxs[size_t(s0_ind)].port.set_cam_et_from_perceive_cam_et(et);
      update_depths(sensor_inds, opt_ctxs, false);
   };

   // ---- fn ----
   auto best_score = std::numeric_limits<real>::max();
   auto counter    = 0;
   auto fn         = [&](const real* params) -> real {
      unpack(params);
      const auto& ctx0    = opt_ctxs[size_t(s0_ind)];
      const auto& f2d0    = ctx0.f2d;
      const int n_spixels = int(f2d0.slic_numlabels);

      const auto& C = et.translation;

      auto score    = 0.0;
      auto px_count = 0;

      // For each super-pixel
      for(auto i = 0; i < n_spixels; ++i) {
         const auto& spixel = f2d0.slic_info[size_t(i)];
         // Get the plane equation from the depth map
         const auto p3 = label_from_depth(ctx0, unsigned(i));

         for(const auto& D : spixel.inliers) {
            const auto y = transfer_left_right(bcam, cu, et, p3, to_vec2(D));
            if(y.is_finite() and lab1.in_bounds(y)) {
               auto dist = cie1976_normalized(lab0(D), lab1(y));
               score += real(dist);
            } else {
               score += cie1976_mean + cie1976_stddev;
            }
            px_count++;
         }
      }

      auto ret = score / real(px_count);
      if(ret < best_score) {
         best_score = ret;
         if(feedback) {
            cout << format("#{:6d}, score = {}, et = {}", counter, ret, str(et))
                 << endl;
         }
      }

      counter++;
      return ret;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;
   int kcount    = 100000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          n_params,
                          &start[0],
                          &xmin[0],
                          reqmin,
                          diffstep,
                          10,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);

   } else {
      method = "nelder-mead";

      vector<real> step(n_params);
      auto X = &step[0];
      for(auto i = 0; i < 3; ++i) *X++ = 0.02;            // 2cm
      for(auto i = 0; i < 3; ++i) *X++ = to_radians(2.0); // 2 degrees

      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  10 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(&xmin[0]);

   if(feedback) {
      cout << format("Feedback for optimizing pos of {}", bcam.camera_id)
           << endl;
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
      cout << format("et0 = {}", str(et0)) << endl;
      cout << format("et  = {}", str(et)) << endl;
      cout << endl;
   }
#endif
}

// ---------------------------------------------------- Optimize Camera Position

OptimizeCameraResult optimize_camera_position(const OptimizeCameraData& data,
                                              const bool feedback)
{
   if(feedback) INFO("Optimize Camera Position");

   for(auto i = 0u; i < data.n_sensors(); ++i) {
      const auto fmt = data.scene_desc.model(int(i)).working_format();
      const auto& im = data.scene_desc.sensor_image[i];
      Expects(im.cols == fmt.x);
      Expects(im.rows == fmt.y);
   }

   OptimizeCameraResult opt_result;

#ifndef USING_OPENGL
   FATAL("opengl not compiled in");
#else
   vector<CameraPosOptContext> opt_ctxs;

   // ---- Make the display list -----
   auto display_list_index
       = make_model_displaylist(*data.scene_desc.cad_model_ptr);
   opt_ctxs = init_optimize_operation(data, feedback);
   for(auto& ctx : opt_ctxs) ctx.display_list_index = display_list_index;

   if(false) {
      Plane p3(0, 0, 1, 0);
      const int bcam_id = 3;
      const Vector2 D(745, 468);
      const auto& bcam = data.scene_desc.bcam_infos[bcam_id];
      CachingUndistortInverse cu;
      cu.init(bcam.M[1]);
      const auto s_ind = data.scene_desc.sensor_lookup(bcam_id, 0);
      const auto& et   = data.scene_desc.aruco_transforms[size_t(s_ind)];
      INFO(format("bcam is {}, [{}, {}]",
                  bcam.camera_id,
                  bcam.M[0].sensor_id(),
                  bcam.M[1].sensor_id()));
      const auto y = transfer_left_right(bcam, cu, et, p3, D);
      cout << format("D = {} -- --> y = {}", str(D), str(y)) << endl;
      FATAL("kBAM");
   }

   // optimize_one_bino_cam(data, 3, opt_ctxs);

   const auto count = 1;
   auto s           = time_thunk([&]() {
      for(auto j = 0; j < count; ++j) update_depths(opt_ctxs);
   });

   cout << format("time = {}s", s / real(count * opt_ctxs.size())) << endl;

   // Export the images
   for(auto i = 0u; i < opt_ctxs.size(); ++i) {
      opt_ctxs[i].read_depth_result_from_fbo();
      opt_ctxs[i].debug_export_gl_result("/tmp/cam-pos/initial");
   }

   // ---- A little test ----
   auto test_plane = [&](string msg, auto cam_ind, auto spixel_ind) {
      const auto& ctx = opt_ctxs[size_t(cam_ind)];
      const auto p3   = label_from_depth(ctx, spixel_ind);
      const auto D    = ctx.f2d.slic_info[spixel_ind].center;
      const auto R    = ctx.model.undistort(Vector3(D.x, D.y, 1.0));

      const auto YY = ctx.distorted_to_world(D);

      const auto et = ctx.port.cam_et();
      const auto S  = ctx.port.regular_eye_to_screen(R);

      const auto C  = et.translation;
      const auto Rr = et.rotation.rotate(R);

      const auto X1 = plane_ray_intersection(p3, C, C + Rr);

      const auto W  = ctx.z_buffer.screen_to_world(ctx.port, S);
      const auto WE = (W - et.translation).normalized();
      const auto XX = plane_ray_intersection(p3, C, C + WE);

      cout << string(70, '-') << endl;
      cout << format(
          "Testing '{}': {} {}", msg, ctx.model.sensor_id(), spixel_ind)
           << endl;
      cout << format("D     = {}", str(D)) << endl;
      cout << format("R     = {}", str(R)) << endl;
      cout << format("eye   = {}", str(et.translation)) << endl;
      cout << format("W     = {}", str(W)) << endl;
      cout << format("...   = {}", str(et.rotation.rotate(R))) << endl;
      cout << format("S     = {}", str(S)) << endl;
      cout << format("p3    = {}", str(p3)) << endl;

      cout << format("XX    = {}", str(XX)) << endl;
      cout << format("YY    = {}", str(YY)) << endl;

      cout << format("X1    = {}", str(X1)) << endl;
      cout << endl;
   };

   if(false) {
      for(auto spixel_ind : slic_str3_interest) test_plane("", 0, spixel_ind);
   }

   // Destroy the display list
   if(display_list_index > 0) destroy_model_displaylist(display_list_index);

#endif
   return opt_result;
}

OptimizeCameraResult optimize_campos_and_model(const OptimizeCameraData& data,
                                               const bool feedback)
{
   OptimizeCameraResult ret;

   INFO("OPTIMIZE CAMERA POSITION");

   return ret;
}

} // namespace perceive
