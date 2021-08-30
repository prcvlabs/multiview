
#pragma once

#include "perceive/io/json-io.hpp"

#include <set>

namespace perceive
{
struct SceneDescriptionInfo;
} // namespace perceive

namespace perceive::calibration
{
struct PhasePlaneData
{
 public:
   struct PlaneInfo
   {
      string name{""s};
      int type{0}; // 0->X, 1->Y, 2->Z, otherwise->Z
      real d{0.0};
      bool do_optimize{false};
      vector<std::set<int>> selected; // one per sensor image

      bool operator==(const PlaneInfo& o) const noexcept;
      bool operator!=(const PlaneInfo& o) const noexcept;

      Plane p3() const noexcept
      {
         switch(type) {
         case 0: return Plane(1, 0, 0, d);
         case 1: return Plane(0, 1, 0, d);
         case 2: return Plane(0, 0, 1, d);
         }
         return Plane(0, 0, 1, d);
      }

      const char* type_str() const noexcept
      {
         switch(type) {
         case 0: return "X";
         case 1: return "Y";
         case 2: return "Z";
         }
         return "Z";
      }
   };

   struct StillInfo
   {
      string camera_key{""s};
      unsigned slic_size{250};
      real slic_compactness{15.0};
      bool do_optimize{false};

      bool operator==(const StillInfo& o) const noexcept;
      bool operator!=(const StillInfo& o) const noexcept;
   };

   struct RayPlanePlaneInfo
   {
      // Always for zero-ith sensor in the camera
      string sensor_key{""s};
      array<int, 3> inds{-1, -1, -1}; // indexes into PlaneInfo
      Vector2 x{0, 0}; // the distorted point sitting on the two planes

      // is plane-plane-plane
      bool is_point() const noexcept { return inds[2] >= 0; }
      bool is_line() const noexcept { return !is_point(); }

      bool operator==(const RayPlanePlaneInfo& o) const noexcept;
      bool operator!=(const RayPlanePlaneInfo& o) const noexcept;
   };

   string scene_key{""s};
   vector<PlaneInfo> p3s;
   vector<StillInfo> s_infos;
   vector<RayPlanePlaneInfo> rp_infos;
   int highlighted_p3{-1};

   bool operator==(const PhasePlaneData& o) const noexcept;
   bool operator!=(const PhasePlaneData& o) const noexcept;

   string to_string() const noexcept;
   string to_json_string() const noexcept;

   bool init(const SceneDescriptionInfo&) noexcept;
};

// ---------------------------------------------------------------- Input/Output

void load(PhasePlaneData& data, const string& fname) noexcept(false);
void save(const PhasePlaneData& data, const string& fname) noexcept(false);
void read(PhasePlaneData& data, const std::string& in) noexcept(false);
void write(const PhasePlaneData& data, std::string& out) noexcept(false);
void read(PhasePlaneData& data, const Json::Value& node) noexcept(false);
void write(const PhasePlaneData& data, Json::Value& node) noexcept(false);

} // namespace perceive::calibration
