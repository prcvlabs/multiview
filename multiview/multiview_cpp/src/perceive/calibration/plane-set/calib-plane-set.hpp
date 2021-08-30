
#pragma once

#include "perceive/io/json-io.hpp"

namespace perceive::calibration
{
struct CalibPlane
{
   string name;
   Plane p3;
   std::vector<int> l_spixel_indices;
   std::vector<int> r_spixel_indices;

   // Planes should be 'x' (wall), 'y' (wall), or 'z' (floor)
   int plane_type = 2; // x==0, y==1, z==2

   std::vector<int>& indices(int ind) noexcept
   {
      return (ind == 0) ? l_spixel_indices : r_spixel_indices;
   }

   const std::vector<int>& indices(int ind) const noexcept
   {
      return (ind == 0) ? l_spixel_indices : r_spixel_indices;
   }

   bool operator==(const CalibPlane&) const noexcept;
   bool operator!=(const CalibPlane&) const noexcept;

   string to_string() const;
   string to_json_string() const;
};

struct CalibPlaneSet
{
   vector<CalibPlane> p3s;
   int selected_index = -1;

   CalibPlaneSet(); // created with a single "floor" plane
   void clear();

   bool operator==(const CalibPlaneSet&) const noexcept;
   bool operator!=(const CalibPlaneSet&) const noexcept;

   string duplicates_report() const;

   string to_string() const;
   string to_json_string() const;
};

void load(CalibPlaneSet& data, const string& fname) noexcept(false);
void save(const CalibPlaneSet& data, const string& fname) noexcept(false);
void read(CalibPlaneSet& data, const std::string& in) noexcept(false);
void write(const CalibPlaneSet& data, std::string& out) noexcept(false);
void read(CalibPlaneSet& data, const Json::Value& node) noexcept(false);
void write(const CalibPlaneSet& data, Json::Value& node) noexcept(false);

// Vector of calib-plane-sets
void load(vector<CalibPlaneSet>& data, const string& fname) noexcept(false);
void save(const vector<CalibPlaneSet>& data,
          const string& fname) noexcept(false);
void read(vector<CalibPlaneSet>& data, const std::string& in) noexcept(false);
void write(const vector<CalibPlaneSet>& data, std::string& out) noexcept(false);
void read(vector<CalibPlaneSet>& data, const Json::Value& node) noexcept(false);
void write(const vector<CalibPlaneSet>& data,
           Json::Value& node) noexcept(false);

} // namespace perceive::calibration
