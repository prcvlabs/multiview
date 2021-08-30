
#include "bcam_init.hpp"
#include "get_xy_mapping.hpp"
#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"
#include <opencv2/opencv.hpp>

namespace perceive::pipeline::get_xy_mapping
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, sensor_index, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};

   ret->scene_result = data.match_result<load_scene_description::Result>(
       "load_scene_description");
   if(!ret->scene_result) {
      LOG_ERR("load_scene_description not found");
      return nullptr;
   }

   const auto& scene_desc{ret->scene_desc()};
   const auto indices     = scene_desc.bcam_lookup(int(params.sensor_index));
   const auto cam_num     = indices[0];
   const auto bcam_sensor = indices[1];

   ret->bcam_result
       = data.match_result<bcam_init::Result>(format("bcam_init[{}]", cam_num));
   if(!ret->bcam_result) {
      LOG_ERR(format("bcam_init[{}] not found", cam_num));
      return nullptr;
   }

   if(is_cancelled()) return nullptr;

   const auto& bino_cam{ret->bcam_result->bcam};

   auto s{time_thunk([&] {
      if(is_cancelled()) return;
      bino_cam.get_sensor_mapxy(unsigned(bcam_sensor), ret->mapx, ret->mapy);
#ifdef WITH_CUDA
      ret->mapx_gpu.upload(ret->mapx);
      ret->mapy_gpu.upload(ret->mapy);
#endif
   })};

   if(params.feedback) {
      print_timing(format("#{} Load mapxy: {}s", params.sensor_index, s));
   }

   return is_cancelled() ? nullptr : ret;
}
} // namespace perceive::pipeline::get_xy_mapping
