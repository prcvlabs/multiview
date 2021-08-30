#include "copy_sensor_images.hpp"
#include "perceive/graphics/hist-equalize.hpp"

namespace perceive::pipeline::load_scene_description
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      // m.push_back(MAKE_META(ThisParams, META_COMPATIBLE, gaze_params, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

bool Params::operator==(const Params& o) const noexcept
{
   return scene_desc == o.scene_desc;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result> Task::execute(const RunData& data,
                                       const Params& params,
                                       std::function<bool()> is_cancelled) const
    noexcept
{
   if(!params.scene_desc)
      FATAL(format("failed to set 'scene_desc' in load scene task"));

   auto ret{make_shared<Result>()};
   ret->scene_desc = params.scene_desc;
   return ret;
}
} // namespace perceive::pipeline::load_scene_description
