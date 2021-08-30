
#pragma once

#include "perceive/geometry/vector.hpp"
#include "json/json.h"

namespace perceive
{
struct ArucoResultInfo
{
   string scene_key{""};
   string reference_sensor{""};

   EuclideanTransform global_transform;
   vector<string> sensor_keys;

   // These still need to be transformed by the global-transform
   vector<EuclideanTransform> aruco_transforms;

   CUSTOM_NEW_DELETE(ArucoResultInfo)

   bool operator==(const ArucoResultInfo& o) const noexcept
   {
      return scene_key == o.scene_key and reference_sensor == o.reference_sensor
             and global_transform == o.global_transform
             and sensor_keys == o.sensor_keys
             and aruco_transforms == o.aruco_transforms;
   }
   bool operator!=(const ArucoResultInfo& o) const noexcept
   {
      return !(*this == o);
   }

   string to_string() const noexcept;
   string to_json_string() const noexcept;

   size_t size() const noexcept { return aruco_transforms.size(); }

   vector<EuclideanTransform> global_aruco_transforms() const
   {
      const auto& gt = global_transform;

      decltype(aruco_transforms) ret(aruco_transforms.size());
      std::transform(cbegin(aruco_transforms),
                     cend(aruco_transforms),
                     begin(ret),
                     [&](const auto& et) { return et * gt; });

      return ret;
   }

   friend string str(const ArucoResultInfo& sd) { return sd.to_string(); }
};

void load(ArucoResultInfo& data, const string& fname) noexcept(false);
void read(ArucoResultInfo& data, const std::string& in) noexcept(false);
void read(ArucoResultInfo& data, const Json::Value& node) noexcept(false);

void save(const ArucoResultInfo& data,
          const std::string& fname) noexcept(false);
void write(const ArucoResultInfo& data, std::string& out) noexcept(false);

} // namespace perceive
