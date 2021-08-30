
#include "load-params.hpp"

namespace perceive
{
// static const Json::Value* find_by_key(const Json::Value* o_ptr,
//                                       const string_view key) noexcept
// {
//    if(o_ptr == nullptr) return nullptr;
//    const auto& o = *o_ptr;
//    if(o.type() != Json::objectValue) return nullptr;
//    if(o.isMember(key.data())) return &o[key.data()];
//    return nullptr;
// }

const Json::Value* find_by_path(const Json::Value& o,
                                const string_view jpath) noexcept(false)
{
   if(o.type() != Json::objectValue) return nullptr;

   string path = jpath.data();
   // string::size_type pos = 0;

   auto o_ptr            = &o;
   string::size_type pos = 0;
   auto next             = path.find_first_of('/');
   while(next != string::npos) {
      path[next] = '\0'; // Replace '/' with '\0', to make a c-string
      if(o_ptr->isMember(&path[pos])) o_ptr = &o_ptr->operator[](&path[pos]);
      pos  = next + 1;
      next = path.find_first_of('/', pos);
   }

   if(o_ptr->isMember(&path[pos])) return &o_ptr->operator[](&path[pos]);

   return nullptr;
}

LocalizationData::Params
read_localization_params(const Json::Value& pipeline_params) noexcept(false)
{
   LocalizationData::Params defaults, p;

   const auto path     = "frame_params/localization/localization_params"s;
   const auto node_ptr = find_by_path(pipeline_params, path);
   if(node_ptr == nullptr) {
      WARN("localization parameters not found!");
      return {};
   }

   p.read_with_defaults(*node_ptr, &defaults, true);

   return p;
}
} // namespace perceive
