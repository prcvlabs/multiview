
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/skeleton/skeleton-3d.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
// --------------------------------------------------- Tweaker GL Render Options

struct GuiRenderOptions final : public MetaCompatible
{
   virtual ~GuiRenderOptions() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool use_image_colors{true};
   bool draw_cams{false};
   bool draw_roi{false};
   bool draw_floor_image{false};
   bool clip_point_cloud{false};
   bool live_C_update{false};
   bool skeleton_debug{false};
   bool draw_pose3d{true};
   bool draw_loaded_tracks{false}; // only if available

   bool draw_axis{false};
   bool draw_grid_xy{false};
   bool draw_grid_xz{false};
   bool draw_grid_yz{false};
   bool draw_fine_grid_xy{false};

   bool do_rotate{false};
   bool do_capture{false};
   bool do_capture_movie{false};

   bool smooth_point_cloud{true};

   real rotation_speed{64.0}; // 64 seconds for one rotation
   real helicopter_theta{to_radians(30)};

   vector<Point2> cam_pts_cloud_ind; // [cam_num, point-cloud-index]

   Vector2 dxy;
   real rotation_z{0.0};

   Vector3 pan_xyz{0.0, 0.0, 0.0};
   Vector3 C{0.0, 0.0, 0.0};   // offset for camera center
   Vector3 saa{0.0, 0.0, 0.0}; // Actually 'saa' difference

   real look_distance{1.0};

   int p2d_sensor = -1; //
   int p2d_index  = -1; // within sensor, could be out of range

   float p3d_height = 2.0f;

   EuclideanTransform make_global_et() const noexcept
   {
      EuclideanTransform et;
      et.translation = Vector3(dxy.x, dxy.y, 0.0);
      et.rotation    = Quaternion(Vector4(0, 0, 1, rotation_z));
      et.scale       = 1.0;
      return et;
   }
};

// ---------------------------------------------------------------- Input/Output

META_READ_WRITE_LOAD_SAVE(GuiRenderOptions);

} // namespace perceive
