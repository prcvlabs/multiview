
#include "bcam_init.hpp"
#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::bcam_init
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, cam_num, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, width, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, height, true));
      m.push_back(MAKE_META(ThisParams, REAL, K_fx, true));
      m.push_back(MAKE_META(ThisParams, REAL, K_fy, true));
      m.push_back(MAKE_META(ThisParams, REAL, K_pptx, true));
      m.push_back(MAKE_META(ThisParams, REAL, K_ppty, true));
      m.push_back(MAKE_META(ThisParams, BOOL, use_calib_roi, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// Params::Params()
// {
//    K       = Matrix3r::Identity();
//    K(0, 0) = K(1, 1) = 200.0;
//    K(0, 2)           = width / 2;
//    K(1, 2)           = height / 2;
// }

Matrix3r Params::K() const noexcept
{
   Matrix3r K = Matrix3r::Identity();
   K(0, 0)    = K_fx;
   K(1, 1)    = K_fy;
   K(0, 2)    = K_pptx;
   K(1, 2)    = K_ppty;
   return K;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};
   auto cam_num{params.cam_num};

   ret->load_result = data.match_result<load_scene_description::Result>(
       "load_scene_description");
   if(!ret->load_result) {
      LOG_ERR("Dependency not found: load_scene_description");
      return nullptr;
   }

   const auto& scene_desc{*ret->load_result->scene_desc};

   if(is_cancelled()) return nullptr;

   // `SceneDescription::init` sets the working format of each sensor from the
   // first frame it reads. Reading a frame here would decode it on every
   // camera, and add it to the still.
   auto get_wh = [&]() -> Point2 {
      const auto& bcam_info = scene_desc.bcam_infos.at(size_t(cam_num));
      Expects(bcam_info.n_sensors() > 0);
      const auto wh = bcam_info.M[0].working_format();
      Expects(wh.x > 0.0 and wh.y > 0.0);
      return Point2{int(wh.x), int(wh.y)};
   };

   auto s = time_thunk([&]() {
      const auto wh = get_wh();
      ret->bcam.init(scene_desc.bcam_infos[cam_num],
                     unsigned(wh.x),
                     unsigned(wh.y),
                     params.K(),
                     params.width,
                     params.height,
                     params.use_calib_roi);
      Expects(params.width == ret->bcam.w());
      Expects(params.height == ret->bcam.h());
      ret->width  = ret->bcam.w();
      ret->height = ret->bcam.h();
   });

   if(params.feedback) {
      print_timing(format("#{} Initialize bino-cam: {}s", cam_num, s));
   }

   if(is_cancelled()) { return nullptr; }

   return ret;
}
} // namespace perceive::pipeline::bcam_init
