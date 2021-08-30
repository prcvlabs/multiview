
#include "aruco-result-info.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/file-system.hpp"

#define This ArucoResultInfo

namespace perceive
{
// ---------------------------------------------- SceneDecriptionInfo::to_string

string This::to_string() const noexcept
{
   Expects(sensor_keys.size() == aruco_transforms.size());

   auto json_encode_str
       = [&](const string& s) -> string { return json_encode(s); };

   std::stringstream ss("");
   for(auto i = 0u; i < sensor_keys.size(); ++i) {
      if(i > 0u) ss << ",\n";
      ss << format(R"V0G0N(      {{
         "sensor-id":   {},
         "translation": [{}, {}, {}],
         "rotation":    [{}, {}, {}, {}]
      {}}})V0G0N",
                   json_encode_str(sensor_keys[i]),
                   str_precise(aruco_transforms[i].translation.x),
                   str_precise(aruco_transforms[i].translation.y),
                   str_precise(aruco_transforms[i].translation.z),
                   str_precise(aruco_transforms[i].rotation.x),
                   str_precise(aruco_transforms[i].rotation.y),
                   str_precise(aruco_transforms[i].rotation.z),
                   str_precise(aruco_transforms[i].rotation.w),
                   "");
   }
   auto result_s = ss.str();

   return format(R"V0G0N(
{{
   "type":               "Aruco Result",

   "scene-key":          {},

   "reference-sensor":   {},
   "global-translation": [{}, {}, {}],
   "global-rotation":    [{}, {}, {}, {}],

   "aruco-result": [
{}
   ]
}}
{})V0G0N",
                 json_encode_str(scene_key),
                 json_encode_str(reference_sensor),
                 str_precise(global_transform.translation.x),
                 str_precise(global_transform.translation.y),
                 str_precise(global_transform.translation.z),
                 str_precise(global_transform.rotation.x),
                 str_precise(global_transform.rotation.y),
                 str_precise(global_transform.rotation.z),
                 str_precise(global_transform.rotation.w),
                 result_s,
                 "");
}

string This::to_json_string() const noexcept { return to_string(); }

// -------------------------------------------------------- Load/Save Read/Write

void load(ArucoResultInfo& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void read(ArucoResultInfo& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);
}

void read(ArucoResultInfo& data, const Json::Value& node) noexcept(false)
{
   std::string err = "";

   { // Reinitialize data
      ArucoResultInfo tmp;
      data = tmp;
   }

   auto init_et = [](const Json::Value& key, string prefix) {
      EuclideanTransform et;
      json_load(get_key(key, format("{}translation", prefix)),
                et.translation);
      json_load(get_key(key, format("{}rotation", prefix)), et.rotation);
      et.scale = 1.0;
      return et;
   };

   auto from_json = [&](const Json::Value& root) {
      // Attempt to locate the model_filename
      json_load(get_key(root, "scene-key"), data.scene_key);
      json_load(get_key(root, "reference-sensor"), data.reference_sensor);
      data.global_transform = init_et(root, "global-");

      auto ares_key = root["aruco-result"];
      if(ares_key.isNull())
         throw std::runtime_error("failed to find 'aruco-result' key");
      if(!ares_key.isArray())
         throw std::runtime_error("'aruco-result' key must be a json array");

      auto N = ares_key.size();
      data.sensor_keys.resize(N);
      data.aruco_transforms.resize(N);
      for(auto i = 0u; i < N; ++i) {
         json_load(get_key(ares_key[i], "sensor-id"), data.sensor_keys[i]);
         data.aruco_transforms[i] = init_et(ares_key[i], "");
      }

      // Now some sanity checkes
      if(std::find(cbegin(data.sensor_keys),
                   cend(data.sensor_keys),
                   data.reference_sensor)
         == cend(data.sensor_keys))
         throw std::runtime_error(format("failed to find reference sensor "
                                         "key '{}' in 'aruco-result' array",
                                         data.reference_sensor));
   };

   try {
      from_json(node);
   } catch(std::logic_error& e) {
      err = strlen(e.what()) == 0 ? "logic error" : e.what();
   } catch(std::runtime_error& e) {
      err = strlen(e.what()) == 0 ? "runtime error" : e.what();
   } catch(std::exception& e) {
      // JSON parse error
      err = strlen(e.what()) == 0 ? "exception" : e.what();
   } catch(...) {
      err = format("unknown error");
   }

   if(err != "") throw std::runtime_error(err);
}

void save(const ArucoResultInfo& data, const std::string& fname) noexcept(false)
{
   std::string data_s;
   write(data, data_s);
   file_put_contents(fname, data_s);
}

void write(const ArucoResultInfo& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

} // namespace perceive
