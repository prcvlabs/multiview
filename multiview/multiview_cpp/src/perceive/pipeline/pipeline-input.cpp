
#include "pipeline-input.hpp"

#include "frame-results.hpp"

#define This PipelineInput

namespace perceive::pipeline
{
// ------------------------------------------------------------------- meta data
//
const vector<MemberMetaData>& This::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      // m.push_back(MAKE_META(This, REAL, frame_fps, true));
      m.push_back(MAKE_META(This, JSON_VALUE, task_params, true));
      // m.push_back(MAKE_META(This, JSON_VALUE, frame_params, true));
      m.push_back(MAKE_META(This, COMPATIBLE_OBJECT, fowlkes_params, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// -------------------------------------------------------- lookup-person-radius
//
real lookup_person_radius(const Json::Value& frame_params)
{
   std::function<const Json::Value*(const string& key,
                                    const Json::Value& value)>
       find_key = [&find_key](const string& key,
                              const Json::Value& value) -> const Json::Value* {
      if(value.type() == Json::objectValue) {
         if(value.isMember(key)) { return &value[key]; }
         for(const auto& child : value) {
            auto val = find_key(key, child);
            if(val != nullptr) return val;
         }
      }
      return nullptr;
   };

   auto node = find_key("person_diameter"s, frame_params);
   if(node == nullptr || !node->isNumeric()) return 0.3;
   return 0.5 * node->asDouble();
}

} // namespace perceive::pipeline
