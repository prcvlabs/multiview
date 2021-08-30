
#include "params.hpp"

namespace perceive::pose_skeleton
{
// ----------------------------------------------------------------- render mode
//
static RenderMode to_render_mode(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return RenderMode::x;
   E(NONE);
   E(CPU);
   E(GPU);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{}' to a RenderMode", val));
}

const char* str(const RenderMode x) noexcept
{
   switch(x) {
#define E(x) \
   case RenderMode::x: return #x;
      E(NONE);
      E(CPU);
      E(GPU);
#undef E
   }
}

RenderMode int_to_render_mode(int val) noexcept
{
   const auto r = RenderMode(val);
   switch(r) {
#define E(x) \
   case RenderMode::x: return RenderMode::x;
      E(NONE);
      E(CPU);
      E(GPU);
   default: return RenderMode::NONE;
#undef E
   }
   return r;
}

// --------------------------------------------------- OpenposeParams::meta-data
//
const vector<MemberMetaData>& OpenposeParams::meta_data() const noexcept
{
#define ThisParams OpenposeParams
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, BOOL, single_threaded, false));

      m.push_back(
          MAKE_META(ThisParams, BOOL, only_one_sensor_per_camera, true));
      m.push_back(MAKE_META(ThisParams, INT, image_patch_size, true));

      m.push_back(MAKE_META(ThisParams, BOOL, pose_enabled, true));

      m.push_back({meta_type::JSON_VALUE,
                   "net_input_size"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      Json::Value x{Json::arrayValue};
                      x.resize(2);
                      x[0] = o.net_input_size.x;
                      x[1] = o.net_input_size.y;
                      return std::any(x);
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o          = *reinterpret_cast<ThisParams*>(ptr);
                      const auto& s    = std::any_cast<const Json::Value>(x);
                      o.net_input_size = Point2(s[0].asInt(), s[1].asInt());
                   }});

      m.push_back(MAKE_META(ThisParams, INT, gpu_start, false));
      m.push_back(MAKE_META(ThisParams, INT, num_gpus, false));
      m.push_back(MAKE_META(ThisParams, INT, scales_num, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, scale_gap, true));
      m.push_back({meta_type::STRING,
                   "render_mode"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.render_mode)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.render_mode   = to_render_mode(s);
                   }});
      m.push_back({meta_type::STRING,
                   "pose_model"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.pose_model)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.pose_model    = to_pose_model(s);
                   }});
      m.push_back(MAKE_META(ThisParams, BOOL, bg_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, body_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, PAF_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, add_part_candidates, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, render_threshold, true));
      m.push_back(MAKE_META(ThisParams, INT, max_n_people, true));
      m.push_back(MAKE_META(ThisParams, BOOL, maximize_recall, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// -----------------------------------------------------------
//
const vector<MemberMetaData>& HyperposeParams::meta_data() const noexcept
{
#define ThisParams HyperposeParams
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, STRING, model_name, true));
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ----------------------------------------------------------- Params::meta-data
//
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, use_hyperpose, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, op_params, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, hp_params, true));
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

} // namespace perceive::pose_skeleton
