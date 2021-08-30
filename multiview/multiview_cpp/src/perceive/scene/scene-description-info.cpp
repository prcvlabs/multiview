
#include "scene-description-info.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/io/versioned-resource.hpp"
#include "perceive/utils/file-system.hpp"

#define This SceneDescriptionInfo

namespace perceive
{
// --------------------------------------------- SceneDecriptionInfo::operator==
bool This::operator==(const SceneDescriptionInfo& o) const noexcept
{
   return store_id == o.store_id and scene_key == o.scene_key
          and aruco_result_key == o.aruco_result_key
          and background_image == o.background_image
          and cad_model_key == o.cad_model_key
          and cad_model_transform == o.cad_model_transform
          and global_transformation == o.global_transformation
          and known_planes == o.known_planes and stills == o.stills
          and bcam_keys == o.bcam_keys and bcam_transforms == o.bcam_transforms
          and entrance_zone == o.entrance_zone and staff_zone == o.staff_zone
          and dead_zones == o.dead_zones and look_zones == o.look_zones
          and ((!hist_bounds.is_finite() and !o.hist_bounds.is_finite())
               or (hist_bounds == o.hist_bounds));
}

bool This::operator!=(const SceneDescriptionInfo& o) const noexcept
{
   return !(*this == o);
}

// ---------------------------------------------- SceneDecriptionInfo::to_string

string This::to_string() const noexcept { return to_json_string(); }

string This::to_json_string() const noexcept
{
   auto json_encode_str
       = [&](const string& s) -> string { return json_encode(s); };

   auto keys_s
       = implode(cbegin(bcam_keys), cend(bcam_keys), ", ", json_encode_str);

   auto pack_euc_transform = [](const auto& t) {
      return format("{{\"translation\": [{}, {}, {}],\n"
                    "                              "
                    "\"rotation\" : [{}, {}, {}, {}],\n"
                    "                              "
                    "\"scale\" : 1.0}}",
                    str_precise(t.translation.x),
                    str_precise(t.translation.y),
                    str_precise(t.translation.z),
                    str_precise(t.rotation.x),
                    str_precise(t.rotation.y),
                    str_precise(t.rotation.z),
                    str_precise(t.rotation.w));
   };

   auto transforms_s = implode(cbegin(bcam_transforms),
                               cend(bcam_transforms),
                               ",\n                              ",
                               pack_euc_transform);

   auto implode_polygon = [](const Polygon& P) {
      return implode(
          cbegin(P), cend(P), ", ", [](const auto& X) -> std::string {
             return format("[{}, {}]", X.x, X.y);
          });
   };

   auto poly_vec_to_s = [&implode_polygon](const vector<Polygon>& zones) {
      return implode(
          cbegin(zones), cend(zones), ", ", [&](const auto& P) -> std::string {
             return format("[{}]", implode_polygon(P));
          });
   };

   auto enterance_s  = implode_polygon(entrance_zone);
   auto staff_zone_s = implode_polygon(staff_zone);
   auto dead_zones_s = poly_vec_to_s(dead_zones);
   auto look_zones_s = poly_vec_to_s(look_zones);

   auto hist_aabb_s
       = (hist_bounds.is_finite())
             ? format("\n   \"hist-aabb\":             [{}, {}, {}, {}],",
                      hist_bounds.left,
                      hist_bounds.top,
                      hist_bounds.right,
                      hist_bounds.bottom)
             : ""s;

   auto known_planes_s = implode(cbegin(known_planes),
                                 cend(known_planes),
                                 ",\n                              ",
                                 [](const auto& np3) {
                                    const auto p3 = np3.second;
                                    return format("{}: [{}, {}, {}, {}]",
                                                  json_encode(np3.first),
                                                  p3.x,
                                                  p3.y,
                                                  p3.z,
                                                  p3.w);
                                 });

   auto stills_s
       = implode(cbegin(stills), cend(stills), ", ", [](const auto& s) {
            return json_encode(s);
         });

   auto scene_key_s           = json_encode_str(scene_key);
   auto aruco_result_key_s    = json_encode_str(scene_key);
   auto cad_model_key_s       = json_encode_str(cad_model_key);
   auto cad_model_transform_s = cad_model_transform.to_json_str();

   return format(
       R"V0G0N(
{{
   "type":                  "SceneDescriptionInfo",
   "store-id":               {},
   "scene-key":              {},
   "cad-model-key":          {},
   "cad-model-transform":    {},{}
   "entrance-zone":         [{}],
   "staff-zone":            [{}],
   "dead-zones":            [{}],
   "look-zones":            [{}],
   "background-image":       {},
   "global-transformation":  {},
   "bcam-keys":             [{}],
   "bcam-transforms":       [{}],
   "known-planes":          [{}],
   "stills":                [{}]
}}
{})V0G0N",
       store_id,
       scene_key_s,
       cad_model_key_s,
       cad_model_transform_s,
       hist_aabb_s,
       enterance_s,
       staff_zone_s,
       dead_zones_s,
       look_zones_s,
       json_encode_str(background_image),
       pack_euc_transform(global_transformation),
       keys_s,
       transforms_s,
       known_planes_s,
       stills_s,
       "");
}

// ------------------------------------------------------------- make-still-name

string This::make_still_name(const int bcam_index) const noexcept
{
   Expects(bcam_index >= 0);
   Expects(bcam_index < int(bcam_keys.size()));

   auto make_part = [&](const string s) {
      const auto rsrc = VersionedResource::make(s);
      if(rsrc.version >= 0) return format("{}_v{}", rsrc.name, rsrc.version);
      return rsrc.name;
   };

   return format("still_{}_{}",
                 make_part(scene_key),
                 make_part(bcam_keys[size_t(bcam_index)]));
}

// -------------------------------------------------- get-background-image-fname

// string This::get_background_image_fname() const noexcept
// {
//    // TODO, this should be an asset??? YES
//    const auto fname = format("{}/multiview/calibration/backgrounds/{}",
//                              perceive_data_dir(),
//                              background_image);
//    return fname;
// }

// ---------------------------------------------------------------- scene-bounds

AABB This::scene_bounds() const noexcept { return hist_bounds; }

// -------------------------------------------------------- Load/Save Read/Write

void load(SceneDescriptionInfo& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void read(SceneDescriptionInfo& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);
}

template<typename T>
void json_load_key(const Json::Value node, const char* key, T& dest)
{
   if(!has_key(node, key))
      throw std::runtime_error(format("key {} does not exist in node", key));

   try {
      json_load(get_key(node, key), dest);
   } catch(...) {
      throw std::runtime_error(format("failed to initialize json data "
                                      "from key '{}'",
                                      key));
   }
}

void read(SceneDescriptionInfo& data, const Json::Value& node) noexcept(false)
{
   std::string err = "";

   { // Reinitialize data
      SceneDescriptionInfo tmp;
      data = tmp;
   }

   auto from_json = [&](const Json::Value& root) {
      // auto bcams_node = get_key(root, "bcam-keys");
      // auto btrans_node = get_key(root, "bcam-transforms");
      // Attempt to locate the model_filename

      json_load(get_key(root, "scene-key"), data.scene_key);
      json_load(get_key(root, "store-id"), data.store_id);

      if(has_key(root, "cad-model-key"))
         json_load(get_key(root, "cad-model-key"), data.cad_model_key);
      if(has_key(root, "cad-model-transform"))
         json_load(get_key(root, "cad-model-transform"),
                   data.cad_model_transform);

      if(has_key(root, "background-image")) {
         json_load(get_key(root, "background-image"), data.background_image);
      }

      if(has_key(root, "hist-aabb")) {
         json_load(get_key(root, "hist-aabb"), data.hist_bounds);
         if(data.hist_bounds.left > data.hist_bounds.right) {
            WARN(format("swapping hist-bounds left and right coords"));
            std::swap(data.hist_bounds.left, data.hist_bounds.right);
         }
         if(data.hist_bounds.top > data.hist_bounds.bottom) {
            WARN(format("swapping hist-bounds top and bottom coords"));
            std::swap(data.hist_bounds.top, data.hist_bounds.bottom);
         }
      }

      data.aruco_result_key = "";
      if(has_key(root, "aruco-result-key"))
         json_load(get_key(root, "aruco-result-key"), data.aruco_result_key);

      data.entrance_zone.clear();
      if(has_key(root, "entrance-zone")) {
         auto key = get_key(root, "entrance-zone");
         if(!key.isArray())
            throw std::runtime_error("expected array for 'entrance-zone'");

         data.entrance_zone.resize(key.size());
         for(auto i = 0u; i < key.size(); ++i)
            json_load(key[i], data.entrance_zone[i]);
      }

      data.staff_zone.clear();
      if(has_key(root, "staff-zone")) {
         auto key = get_key(root, "staff-zone");
         if(!key.isArray())
            throw std::runtime_error("expected array for 'staff-zone'");

         data.staff_zone.resize(key.size());
         for(auto i = 0u; i < key.size(); ++i)
            json_load(key[i], data.staff_zone[i]);
      }

      auto load_vec_polygon = [](const Json::Value& node,
                                 const char* key_s,
                                 vector<SceneDescriptionInfo::Polygon>& zones) {
         zones.clear();
         if(has_key(node, key_s)) {
            auto key = get_key(node, key_s);
            if(!key.isArray())
               throw std::runtime_error(
                   format("expected array for '{}'", key_s));

            zones.resize(key.size());
            for(auto i = 0u; i < key.size(); ++i) {
               const auto& src = key[i];
               auto& zone      = zones[i];

               if(!src.isArray())
                  throw std::runtime_error(
                      format("expected array-or-arrays for '{}'", key_s));
               zone.resize(src.size());
               for(auto j = 0u; j < src.size(); ++j) json_load(src[j], zone[j]);
            }
         }
      };

      load_vec_polygon(root, "dead-zones", data.dead_zones);
      load_vec_polygon(root, "look-zones", data.look_zones);

      data.global_transformation = EuclideanTransform{};
      if(has_key(root, "global-transformation")) {
         json_load_key(
             root, "global-transformation", data.global_transformation);
      }

      auto bcam_keys       = get_key(root, "bcam-keys");
      auto bcam_transforms = get_key(root, "bcam-transforms");

      if(!bcam_keys.isArray())
         throw std::runtime_error("expecting JSON array value for "
                                  "'bcam-keys'");
      if(!bcam_transforms.isArray())
         throw std::runtime_error("expecting JSON array value for "
                                  "'bcam-transforms'");

      if(bcam_keys.size() != bcam_transforms.size())
         throw std::runtime_error(format("'bcam-keys' (size = {}) must be "
                                         "the same size as "
                                         "'bcam-transforms' (size = {})",
                                         bcam_keys.size(),
                                         bcam_transforms.size()));

      const auto N = bcam_keys.size();
      data.bcam_keys.resize(N);
      data.bcam_transforms.resize(N);
      for(auto i = 0u; i < N; ++i) {
         json_load(bcam_keys[i], data.bcam_keys[i]);
         json_load(bcam_transforms[i], data.bcam_transforms[i]);
      }

      {
         // Apply the global transformation... yes this is a bit sloppy,
         // but is an alternative to potential error-prone refactoring...
         const auto et = data.global_transformation;
         // data.global_transformation = EuclideanTransform{};
         for(auto& cam_et : data.bcam_transforms) { cam_et = cam_et * et; }
      }

      // Known planes
      data.known_planes.clear();
      if(has_key(root, "known-planes")) {
         auto jplanes = get_key(root, "known-planes");

         if(jplanes.type() == Json::ValueType::objectValue) {
            data.known_planes.clear();
            for(auto name : jplanes.getMemberNames()) {
               Plane p3;
               json_load(jplanes[name], p3);
               data.known_planes.emplace_back(name, p3);
            }

         } else if(jplanes.type() == Json::ValueType::arrayValue) {
            // A flat array of planes. Names are deduced.
            data.known_planes.resize(jplanes.size());
            for(auto i = 0u; i < jplanes.size(); ++i) {
               data.known_planes[i].first = format("#{}", i);
               json_load(jplanes[i], data.known_planes[i].second);
            }
         } else {
            throw std::runtime_error("'known-planes' malformed");
         }
      }

      data.stills.clear();

      if(has_key(root, "stills")) {
         auto stills_arr = get_key(root, "stills");
         if(stills_arr.size() != 0) {
            if(stills_arr.size() != N)
               throw std::runtime_error(
                   format("expected {} elements in 'stills' "
                          "key, but found {}!",
                          N,
                          stills_arr.size()));
            data.stills.resize(N);
            for(auto i = 0u; i < data.stills.size(); ++i)
               json_load(stills_arr[i], data.stills[i]);
         }
      }
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

void save(const SceneDescriptionInfo& data,
          const std::string& fname) noexcept(false)
{
   std::string data_s;
   write(data, data_s);
   file_put_contents(fname, data_s);
}

void write(const SceneDescriptionInfo& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

} // namespace perceive
