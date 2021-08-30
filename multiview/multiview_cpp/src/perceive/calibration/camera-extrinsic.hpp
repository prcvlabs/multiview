
#pragma once

#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/scene/aruco-result-info.hpp"

namespace perceive
{
EuclideanTransform estimate_camera_extrinsics(
    const DistortionModel& M, // make sure working format is set
    const Vector3& estimated_center,
    const vector<Vector3>& world_coords, // world coords
    const vector<Vector2>& image_coords, // image coords
    real& reproj_error,
    const bool feedback,
    const bool super_feedback = false);

EuclideanTransform estimate_camera_extrinsics(
    const DistortionModel& M, // make sure working format is set
    const Vector3& min_xyz,   // 3D volume in which to find camera
    const Vector3& max_xyz,
    const vector<Vector3>& world_coords, // world coords
    const vector<Vector2>& image_coords, // image coords
    real& reproj_error,
    const bool feedback,
    const bool super_feedback = false);

// ------------------------------------ For estimating camera positions

struct EstimateCamExtrinsicInfo
{
   struct SensorInfo
   {
      string sensor_id{""};
      bool is_regular_sensor{false};
      Matrix3r K;
      Vector3 estimated_center{Vector3::nan()};
      vector<string> coord_names;
      vector<Vector3> world_coords;
      vector<Vector2> image_coords;
      Vector2 image_format;
      string image_filename{""};
   };

   struct WorldCoord
   {
      string name;
      Vector3 pos;
      vector<int> plane_ids; // if the coord sits on a plane
   };

   struct PlaneInfo
   {
      string name;
      Plane p3;
   };

   string scene_key;
   vector<SensorInfo> sensors;
   vector<WorldCoord> coords;
   vector<PlaneInfo> planes;
   unsigned ref_ind{0};
   bool feedback{true};

   Vector3 lookup(const string& s) const
   {
      for(auto i = 0u; i < coords.size(); ++i)
         if(coords[i].name == s) return coords[i].pos;
      throw std::runtime_error(format("world-coord not found: {}", s));
      return Vector3(0, 0, 0);
   }

   // Returns -1 if 's' isn't in sensors
   int sensor_ind(const string& s) const
   {
      auto ii = std::find_if(cbegin(sensors),
                             cend(sensors),
                             [&](const auto& S) { return S.sensor_id == s; });
      return (ii == cend(sensors)) ? -1
                                   : int(std::distance(cbegin(sensors), ii));
   }
};

void load(EstimateCamExtrinsicInfo& data, const string& fname) noexcept(false);
void save(const EstimateCamExtrinsicInfo& data,
          const string& fname) noexcept(false);
void read(EstimateCamExtrinsicInfo& data,
          const std::string& in) noexcept(false);
void write(const EstimateCamExtrinsicInfo& data,
           std::string& out) noexcept(false);
void read(EstimateCamExtrinsicInfo& data,
          const Json::Value& node) noexcept(false);

ArucoResultInfo estimate_extrinsic(const EstimateCamExtrinsicInfo& info);

void run_estimate_extrinsic(const string& input_file,
                            const string& output_file,
                            const bool mega_opt,
                            const bool feedback);

} // namespace perceive
