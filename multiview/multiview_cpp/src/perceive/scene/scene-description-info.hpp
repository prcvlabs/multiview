
#pragma once

#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"
#include "json/json.h"

namespace perceive
{
struct SceneDescriptionInfo
{
   using Polygon = vector<Vector2>;

   unsigned store_id{0};
   string scene_key{""};
   string aruco_result_key{""};
   string cad_model_key{""};
   string background_image{""};
   EuclideanTransform cad_model_transform;
   EuclideanTransform global_transformation;
   vector<std::pair<string, Plane>> known_planes;
   vector<string> stills;
   vector<string> bcam_keys;
   vector<EuclideanTransform> bcam_transforms;
   Polygon entrance_zone; // a polygon
   Polygon staff_zone;    // a polygon
   vector<Polygon> dead_zones;
   vector<Polygon> look_zones;
   AABB hist_bounds{AABB::nan()}; // if not set, then uses cad-model

   CUSTOM_NEW_DELETE(SceneDescriptionInfo)

   bool operator==(const SceneDescriptionInfo& o) const noexcept;
   bool operator!=(const SceneDescriptionInfo& o) const noexcept;

   string to_string() const noexcept;
   string to_json_string() const noexcept;

   string make_still_name(const int bcam_index) const noexcept;
   // string get_background_image_fname() const noexcept;
   AABB scene_bounds() const noexcept;

   friend string str(const SceneDescriptionInfo& sd) { return sd.to_string(); }
};

void load(SceneDescriptionInfo& data, const string& fname) noexcept(false);
void read(SceneDescriptionInfo& data, const std::string& in) noexcept(false);
void read(SceneDescriptionInfo& data, const Json::Value& node) noexcept(false);

void save(const SceneDescriptionInfo& data,
          const std::string& fname) noexcept(false);
void write(const SceneDescriptionInfo& data, std::string& out) noexcept(false);

} // namespace perceive
