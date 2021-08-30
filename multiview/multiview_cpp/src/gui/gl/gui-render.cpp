
#include "gui-render.hpp"

#include "render-skeleton2d-result.hpp"

#include "gui/app-state.hh"

#include "perceive/cost-functions/smooth-point-cloud.hpp"
#include "perceive/foundation.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

#include "perceive/pipeline/nodes/disp_map_update.hpp"

#ifdef USING_OPENGL
#include "gl/gl-utils.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define This GuiRenderOptions

namespace perceive
{
static string process_key(const string key)
{
   auto ret                 = key;
   auto pos                 = 0u;
   auto found_nonzero_digit = false;
   for(auto i = 0u; i < key.size(); ++i) {
      if(!std::isdigit(key[i])) continue;
      if(key[i] == '0' and !found_nonzero_digit) continue;
      found_nonzero_digit = true;
      ret[pos++]          = key[i];
   }
   while(ret.size() > pos) ret.pop_back();
   return format("S {:s}", ret);
}

static const Sprite& cam_sprite(const real size)
{
   // does not need to be thread safe
   static shared_ptr<const Sprite> s
       = make_shared<Sprite>(make_camera_sprite(size));
   return *s;
}

static void render_point_cloud(const GuiRenderOptions& opts,
                               const EuclideanTransform& e0,
                               const PointCloud& pt_cloud,
                               const AABB& bounds,
                               const cv::Mat& ref_undistort) noexcept
{
   const auto& Xs       = pt_cloud.Xs;
   const auto& xy       = pt_cloud.xy;
   const unsigned N     = unsigned(Xs.size());
   const unsigned ref_w = ref_undistort.cols;
   const unsigned ref_h = ref_undistort.rows;

   for(unsigned i = 0; i < N; ++i) {
      const auto& X = Xs[i];
      const auto& x = xy[i];

      if(opts.clip_point_cloud and bounds.is_finite()) {
         // auto Y = et.apply(global_et.apply(X));
         const auto Y = e0.apply(X);
         if(!bounds.contains(Y.x, Y.y)) continue;
      }

      if(opts.use_image_colors and unsigned(x.x) < ref_w
         and unsigned(x.y) < ref_h) {
         auto color = ref_undistort.at<cv::Vec3b>(cv::Point(x.x, x.y));
         glColor3d(color[2] / 255.0, color[1] / 255.0, color[0] / 255.0);
      }
      glVertex3dv(X.ptr());
   }
}

// ----------------------------------------------------- gui render points cloud

void gui_render_point_cloud(const GuiRenderOptions& opts,
                            const pipeline::localization::Result* loc_ptr,
                            const AABB& bounds)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   const auto scene_desc = app_state()->scene_desc();
   const auto n_cameras = (scene_desc == nullptr) ? 0 : scene_desc->n_cameras();
   const auto global_et = opts.make_global_et();
   const int app_current_cam = app_state()->current_camera();

   auto gl_kolour = [](const uint32_t k) {
      const auto r = kolour_to_vector3(k);
      glColor3d(r.x, r.y, r.z);
   };

   auto gl_polygon_vertices = [](const vector<Vector2>& polygon, const real z) {
      for(const auto x : polygon) { glVertex3d(x.x, x.y, z); }
   };

   auto draw_polygon
       = [&](const vector<Vector2>& polygon, const real z, const uint32_t k) {
            if(polygon.size() > 2) {
               glDisable(GL_CULL_FACE);
               glBegin(GL_POLYGON);
               gl_kolour(k);
               gl_polygon_vertices(polygon, z);
               glEnd();
            }
         };

   auto draw_line_polygon
       = [&](const vector<Vector2>& polygon, const real z, const uint32_t k) {
            if(polygon.size() > 2) {
               glEnable(GL_LINE_SMOOTH);
               glLineWidth(6.0f);
               glDisable(GL_CULL_FACE);
               gl_kolour(k);
               glBegin(GL_LINE_LOOP);
               gl_polygon_vertices(polygon, z);
               glEnd();
               glLineWidth(1.0f);
            }
         };

   auto calc_cam_et = [&](const int cam_num) {
      return scene_desc->cam_transforms.at(cam_num);
   };

   glPushMatrix();

   if(opts.pan_xyz.is_finite()) {
      auto rot_z       = Quaternion(Vector4(0, 0, 1, opts.rotation_z));
      const auto pan_e = EuclideanTransform{
          Vector3{opts.pan_xyz.x, opts.pan_xyz.y, opts.pan_xyz.z},
          Quaternion{}};
      gl_mult_matrix(EuclideanTransform(Vector3(0, 0, 0), rot_z) * pan_e);
   }

   if(opts.draw_fine_grid_xy) {
      const auto l = 30.0;
      glLineStipple(1, 0x3F07);
      glEnable(GL_LINE_STIPPLE);
      glBegin(GL_LINES);
      glColor4d(0., 1., 1., 0.5);
      for(real i = -30.0; i <= 30.0; i += 0.1) {
         if(std::round(i) == i) continue;
         glVertex3d(-l, i, 0.);
         glVertex3d(l, i, 0.);
      }
      for(real i = -30.0; i <= 30.0; i += 0.1) {
         if(std::round(i) == i) continue;
         glVertex3d(i, -l, 0.);
         glVertex3d(i, l, 0.);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
   }

   if(opts.draw_grid_xy || opts.draw_fine_grid_xy) { // XY plane
      const auto l = 30.0;
      glLineStipple(1, 0xfefe);
      glEnable(GL_LINE_STIPPLE);
      glBegin(GL_LINES);
      glColor4d(1., 1., 0., 0.5);
      for(auto i = -30; i <= 30; ++i) {
         glVertex3d(-l, i, 0.);
         glVertex3d(l, i, 0.);
      }
      for(auto i = -30; i <= 30; ++i) {
         glVertex3d(i, -l, 0.);
         glVertex3d(i, l, 0.);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
   }

   if(opts.draw_grid_xz) { // XZ plane
      const auto l = 10.0;
      glLineStipple(1, 0x3F07);
      glEnable(GL_LINE_STIPPLE);
      glBegin(GL_LINES);
      glColor4d(1., 0., 1., 0.5);
      for(auto i = 0; i <= 10; ++i) {
         glVertex3d(0., 0., i);
         glVertex3d(l, 0., i);
      }
      for(auto i = 0; i <= 10; ++i) {
         glVertex3d(i, 0., 0.);
         glVertex3d(i, 0., l);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
   }

   if(opts.draw_grid_yz) { // YZ plane
      const auto l = 10.0;
      glLineStipple(1, 0x3F07);
      glEnable(GL_LINE_STIPPLE);
      glBegin(GL_LINES);
      glColor4d(0., 1., 1., 0.5);
      for(auto i = 0; i <= 10; ++i) {
         glVertex3d(0., 0., i);
         glVertex3d(0., l, i);
      }
      for(auto i = 0; i <= 10; ++i) {
         glVertex3d(0., i, 0.);
         glVertex3d(0., i, l);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
   }

   if(opts.draw_axis) {
      auto l = 3.0;
      glBegin(GL_LINES);
      glColor3d(1., 0., 0.);
      glVertex3d(0., 0., 0.);
      glVertex3d(l, 0., 0.);
      glColor3d(0., 1., 0.);
      glVertex3d(0., 0., 0.);
      glVertex3d(0., l, 0.);
      glColor3d(0., 0., 1.);
      glVertex3d(0., 0., 0.);
      glVertex3d(0., 0., l);
      glEnd();
   }

   if(scene_desc != nullptr and opts.draw_roi) {
      // Draw the bounding AABB
      if(scene_desc->scene_info.hist_bounds.is_finite())
         draw_line_polygon(
             scene_desc->scene_info.hist_bounds.to_polygon(), -0.001, k_yellow);

      // Draw the entrance zone
      for(const auto& zone : scene_desc->scene_info.dead_zones) {
         draw_line_polygon(zone, 0.001, k_hot_pink);
         draw_line_polygon(zone, -0.003, k_hot_pink);
      }

      {
         int counter = 0;
         for(const auto& zone : scene_desc->scene_info.look_zones) {
            const auto k = colour_set_2(counter++);
            draw_line_polygon(zone, 0.001, k);
            draw_line_polygon(zone, -0.003, k);
         }
      }

      draw_line_polygon(scene_desc->scene_info.entrance_zone, -0.000, k_red);
      draw_line_polygon(scene_desc->scene_info.entrance_zone, -0.002, k_red);
   }

   if(scene_desc != nullptr and opts.draw_floor_image
      and scene_desc->has_background_image()) {
      Expects(scene_desc->background_tex_id() != 0);
      glBindTexture(GL_TEXTURE_2D, scene_desc->background_tex_id());

      //
      const auto bounds = scene_desc->scene_info.scene_bounds();
      if(bounds.is_finite()) {
         const real z = -0.001;
         const real t = scene_desc->background_image_w_tex_coord;
         const real s = scene_desc->background_image_h_tex_coord;

         glEnable(GL_TEXTURE_2D);
         glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

         glBegin(GL_QUADS);
         glTexCoord2f(0.0, 0.0);
         glVertex3d(bounds.left, bounds.top, z);
         glTexCoord2f(t, 0.0);
         glVertex3d(bounds.right, bounds.top, z);
         glTexCoord2f(t, s);
         glVertex3d(bounds.right, bounds.bottom, z);
         glTexCoord2f(0.0, s);
         glVertex3d(bounds.left, bounds.bottom, z);
         glEnd();
         glDisable(GL_TEXTURE_2D);
      }
   }

   { // Drawing cameras....
      const auto floor_p3 = Plane{0.0, 0.0, 1.0, 0.0};

      auto draw_cam = [&](const EuclideanTransform& et, const string& key) {
         const auto cam_size = 0.05;
         glPushMatrix();
         gl_mult_matrix(et);
         render_gl(cam_sprite(cam_size));

         if(true) { // Draw the ray...
            // get the distance from camera to the floor (down the ray)
            const auto C       = et.translation;
            const auto ray     = et.rotation.apply(Vector3{0.0, 0.0, 1.0});
            const auto ray_len = plane_ray_intersection_t(floor_p3, C, C + ray);

            const auto C0 = Vector3(0.0, 0.0, 0.0);
            const auto C1 = Vector3(0.0, 0.0, ray_len);
            glColor3d(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            glVertex3dv(C0.ptr());
            glVertex3dv(C1.ptr());
            glEnd();
         }

         {
            auto small = process_key(key);
            auto dims  = gl_render_text_sz(small);
            // Write this across the top..
            glPushMatrix(); // PUSH
            glTranslated(0, 0, -cam_size - 1e-3);
            glRotated(180.0, 0.0, 0.0, 1.0);
            const auto s = 0.8 * cam_size;
            glScaled(s, s, 1.0);
            glTranslated(-dims.x * 0.5, -dims.y * 0.5, 0.0);
            gl_render_text(small);
            glPopMatrix(); // POP
         }

         glPopMatrix();
      };

      if(scene_desc and opts.draw_cams) {
         for(auto i = 0; i < n_cameras; ++i) {
            const auto et0 = calc_cam_et(i);
            const auto et1 = scene_desc->bcam_infos[i].make_et1(et0);

            const auto id0 = scene_desc->sensor_lookup(i, 0);
            const auto id1 = scene_desc->sensor_lookup(i, 1);

            draw_cam(et0, scene_desc->sensor_ids[id0]);
            draw_cam(et1, scene_desc->sensor_ids[id1]);

            if(true) { // Draw the circle on the ground
               glPushMatrix();
               Vector3 C = et0.translation;
               C.z       = 0.0;
               glColor3d(0.0, 0.0, 1.0);
               gl_render_circle(C, 0.10, 20);
               glPopMatrix();
            }
         }
      }
   }

   auto render_p3d = [&](const Vector3& C0, real r, real theta, uint32_t k) {
      const auto C = C0 + Vector3{0.0, 0.0, 0.02};
      const auto Z = Vector3(0, 0, 1);
      const auto e = 0.13;
      const auto K = kolour_to_vector3(k);
      glColor3d(K.x, K.y, K.z);
      glLineWidth(4.0);
      gl_render_circle(C, Z, r - 0.00, 100);
      gl_render_circle(C, Z, r - 0.01, 100);

      const auto dxy = Vector3(cos(theta), sin(theta), 0);
      const real phi = acos(r / (r + e));

      const auto W = C + (r + e) * dxy;
      const auto A = C + r * Vector3(cos(theta - phi), sin(theta - phi), 0);
      const auto B = C + r * Vector3(cos(theta + phi), sin(theta + phi), 0);

      glBegin(GL_LINE_STRIP);
      glVertex3dv(A.ptr());
      glVertex3dv(W.ptr());
      glVertex3dv(B.ptr());
      glEnd();
      glLineWidth(1.0);
   };

   if(scene_desc && opts.draw_loaded_tracks == true) {
      // We only need this if (opts.draw_loaded_tracks == true)
      const int frame_no           = app_state()->frame_no();
      const auto& movie_params     = app_state()->movie_params();
      const auto& fw_ret           = movie_params.visualization_preloaded_data;
      const vector<Track>& tts     = fw_ret.tracks;
      const TracksIndex& tt_lookup = movie_params.visualization_index;
      const auto& bounds           = scene_desc->scene_info.hist_bounds;

      const auto top_left
          = (fw_ret.top_left.norm() == 0.0)
                ? bounds.top_left()
                : fw_ret.top_left; // What's the actual top-left?
      const auto hist_sz = (fw_ret.hist_sz == 0.00) ? 0.10 : fw_ret.hist_sz;

      if(bounds.area() <= 0.0 or !bounds.is_finite()) {
         LOG_ERR(format("scene-info hist-bounds not set up!"));
      } else {
         if(tt_lookup.has_frame(frame_no)) {
            for(const auto& item : tt_lookup.frame_data(frame_no)) {
               const auto [tt_ptr, tp_ptr] = tt_lookup.get(tts, item);
               if(tt_ptr == nullptr or tp_ptr == nullptr) {
                  LOG_ERR(format("failed to load item {{}, {}}",
                                 item.track_index,
                                 item.tp_index));
                  continue;
               }

               const auto& tt = *tt_ptr;
               const auto& tp = *tp_ptr;
               const auto C   = FloorHistogram::unproject_hist_xy(
                   Point2(tp.x, tp.y), top_left, hist_sz);
               const auto kolour = colour_set_5(tt.id);

               render_p3d(C, 0.3, tp.gaze_direction, kolour);

               // Are we connected with a "look" zone?
               const auto theta = tp.gaze_direction;
               const auto dxy   = Vector2(cos(theta), sin(theta));
               const auto c     = Vector2(C.x, C.y);
               const auto d     = c + opts.look_distance * dxy;

               for(const auto& zone : scene_desc->scene_info.look_zones) {
                  const auto e = line_segment_polygon_isect(
                      c, d, cbegin(zone), cend(zone));
                  if(e.is_finite()) {
                     if((frame_no / 2) % 2 == 0) {
                        glLineWidth(4.0);
                        draw_polygon(zone, 0.001, kolour);
                        glLineWidth(1.0);
                        // draw_polygon(zone, -0.003, kolour);
                     } else {
                        glBegin(GL_LINE_LOOP);
                        glLineWidth(4.0);
                        gl_kolour(kolour);
                        gl_polygon_vertices(zone, 0.001);
                        glLineWidth(1.0);
                        glEnd();
                     }
                     glBegin(GL_LINES);
                     glLineWidth(3.0);
                     glVertex3d(c.x, c.y, 0.002);
                     glVertex3d(e.x, e.y, 0.002);
                     glLineWidth(1.0);
                     glEnd();
                  }
               }
            }
         }
      }
   }

   if(true) { // Draw point clouds
      auto draw_pt_cloud = [&](int cam_num) {
         if(scene_desc == nullptr) return;
         if(app_state()->frame_results() == nullptr) return;
         auto pipeline = app_state()->frame_results();

         const int sensor_ind = scene_desc->sensor_lookup(cam_num, 0);

         if(unsigned(cam_num) >= pipeline->disp_map_update.size()) return;
         if(unsigned(cam_num) >= scene_desc->cam_transforms.size()) return;
         if(unsigned(sensor_ind) >= pipeline->calc_rectified.size()) return;

         shared_ptr<const pipeline::disp_map_update::Result> disp_ret_ptr
             = pipeline->disp_map_update[cam_num]->try_get_casted_result();
         const auto rect_ret_ptr
             = pipeline->calc_rectified[sensor_ind]->try_get_casted_result();
         if(disp_ret_ptr == nullptr) return;
         if(rect_ret_ptr == nullptr) return;

         const auto& pt_cloud = disp_ret_ptr->point_cloud;
         const auto& et       = scene_desc->cam_transforms[cam_num];

         const auto e0 = calc_cam_et(cam_num);

         glPushMatrix();

         gl_mult_matrix(e0);

         const bool feedback = false;

         glColor3d(1.0, 1.0, 1.0);
         glBegin(GL_POINTS);

         render_point_cloud(
             opts, e0, *pt_cloud, bounds, rect_ret_ptr->rectified);

         glEnd();
         glPopMatrix();
      };

      for(const auto& x : opts.cam_pts_cloud_ind) draw_pt_cloud(x.x);
   }

   if(scene_desc && opts.skeleton_debug) { // Draw skeleton-debug
      const int n_sensors = scene_desc->n_sensors();
      if(loc_ptr != nullptr) {
         const auto ldat = loc_ptr->data;
         int counter     = 0;
         for(const auto& sensor_p2ds : ldat.p2ds) {
            int p2d_ind = 0;
            for(const auto& p2d_info : sensor_p2ds) {
               const auto k     = colour_set_4(counter);
               const auto alpha = 0.5f;
               const auto& s3d  = p2d_info.p2d_ptr->best_3d_result();
               const auto theta = p2d_info.p2d_ptr->theta(); // gaze
               const auto hgt   = s3d.height();
               gl_render_3d_cylinder(s3d, hgt, p2d_ind, k, alpha);
               gl_render_3d_skeleton(s3d, hgt, theta, k);
               ++counter;
               ++p2d_ind;
            }
         }
      }
   }

   flush_transparent_fragments();

   glPopMatrix();
#endif
}

} // namespace perceive
