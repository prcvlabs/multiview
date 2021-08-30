
#include "stdinc.hpp"

#include "gui-render-options.hpp"

#include "perceive/foundation.hpp"
#include "perceive/graphics/sprite.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

#ifdef USING_OPENGL
#include "gl/gl-utils.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define This GuiRenderOptions

namespace perceive
{
const vector<MemberMetaData>& This::meta_data() const noexcept
{
#define ThisParams GuiRenderOptions
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, use_image_colors, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_cams, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_roi, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_floor_image, true));
      m.push_back(MAKE_META(ThisParams, BOOL, clip_point_cloud, true));
      m.push_back(MAKE_META(ThisParams, BOOL, live_C_update, true));
      m.push_back(MAKE_META(ThisParams, BOOL, skeleton_debug, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_pose3d, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_loaded_tracks, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_axis, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_grid_xy, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_grid_xz, true));
      m.push_back(MAKE_META(ThisParams, BOOL, draw_grid_yz, true));
      m.push_back(MAKE_META(ThisParams, BOOL, do_rotate, true));
      m.push_back(MAKE_META(ThisParams, BOOL, do_capture, true));
      m.push_back(MAKE_META(ThisParams, BOOL, do_capture_movie, true));
      m.push_back(MAKE_META(ThisParams, BOOL, smooth_point_cloud, true));

      m.push_back(MAKE_META(ThisParams, REAL, rotation_speed, true));
      m.push_back(MAKE_META(ThisParams, REAL, helicopter_theta, true));

      m.push_back({meta_type::JSON_VALUE, // vector<Point2>
                   "cam_pts_cloud_ind"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      Json::Value z{Json::arrayValue};
                      z.resize(unsigned(o.cam_pts_cloud_ind.size()));
                      for(auto i = 0u; i < z.size(); ++i)
                         z[i] = json_save(o.cam_pts_cloud_ind[i]);
                      return std::any(z);
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o = *reinterpret_cast<ThisParams*>(ptr);
                      const Json::Value& z
                          = std::any_cast<const Json::Value>(x);
                      if(z.type() != Json::arrayValue)
                         throw std::runtime_error("expected Json::array");
                      o.cam_pts_cloud_ind.resize(z.size());
                      for(auto i = 0u; i < z.size(); ++i)
                         json_load(z[i], o.cam_pts_cloud_ind[i]);
                   }});

      m.push_back(MAKE_META(ThisParams, VECTOR2, dxy, true));
      m.push_back(MAKE_META(ThisParams, REAL, rotation_z, true));

      m.push_back(MAKE_META(ThisParams, VECTOR3, pan_xyz, true));
      m.push_back(MAKE_META(ThisParams, VECTOR3, C, true));
      m.push_back(MAKE_META(ThisParams, VECTOR3, saa, true));

      m.push_back(MAKE_META(ThisParams, REAL, look_distance, true));

      m.push_back(MAKE_META(ThisParams, INT, p2d_sensor, true));
      m.push_back(MAKE_META(ThisParams, INT, p2d_index, true));

      m.push_back(MAKE_META(ThisParams, FLOAT, p3d_height, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

} // namespace perceive
