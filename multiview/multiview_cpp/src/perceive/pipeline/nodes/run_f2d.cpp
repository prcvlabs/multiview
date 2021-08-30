
#include "run_f2d.hpp"

#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::run_f2d
{
// ------------------------------------------------------------------- meta data

const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, sensor_index, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, f2d, true));
      return m;
   };
#undef ThisParams
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};

   ret->p = params;

   ret->image_result = data.match_result<input_images::Result>(
       format("input_images[{}]", params.sensor_index));
   if(!ret->image_result) {
      LOG_ERR("Cant find dependency: input_images");
      return nullptr;
   }

   ret->lab_conversion = data.match_result<convert_to_lab::Result>(
       format("convert_to_lab[{}]", params.sensor_index));
   if(!ret->lab_conversion) {
      LOG_ERR("Cant find dependency: convert_to_lab");
      return nullptr;
   }

   auto& images{*ret->image_result};
   const auto& scene_desc{ret->scene_desc()};

   if(is_cancelled()) return nullptr;

   auto s{time_thunk([&] {
      const auto lab_still
          = scene_desc.sensor_LAB_still(int(params.sensor_index));
      init_2d_features(ret->f2d,
                       *ret->lab_conversion->argb_image,
                       *ret->lab_conversion->lab_image,
                       images.grey,
                       &lab_still,
                       params.f2d,
                       is_cancelled,
                       cuda_slic_context);
   })};

   if(params.feedback) {
      print_timing(format("#{} 2d features: {}s (slic = {}s)",
                          params.sensor_index,
                          s,
                          ret->f2d.init_slic_s));
   }

   if(is_cancelled()) { return nullptr; }

   return ret;
}
} // namespace perceive::pipeline::run_f2d
