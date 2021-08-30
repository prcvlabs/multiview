
#include "calib-plane-set.hpp"

#include "perceive/utils/file-system.hpp"

#define This CalibPlaneSet

namespace perceive::calibration
{
// ----------------------------------------------------------------- calib-plane

bool CalibPlane::operator==(const CalibPlane& o) const noexcept
{
   return l_spixel_indices == o.l_spixel_indices
          and r_spixel_indices == o.r_spixel_indices and p3 == o.p3;
}

bool CalibPlane::operator!=(const CalibPlane& o) const noexcept
{
   return !(*this == o);
}

string CalibPlane::to_string() const { return to_json_string(); }

string CalibPlane::to_json_string() const
{
   auto int_arr_s = [](const auto& v) {
      return implode(begin(v), end(v), ", ", [](auto x) { return str(x); });
   };

   auto p3_from_plane_type = [&](int plane_type, real w) {
      auto in_p3 = Plane(0, 0, 0, w);
      if(plane_type == 0) in_p3.xyz() = Vector3(1, 0, 0);
      if(plane_type == 1) in_p3.xyz() = Vector3(0, 1, 0);
      if(plane_type == 2) in_p3.xyz() = Vector3(0, 0, 1);
      return in_p3;
   };
   auto in_p3 = p3_from_plane_type(plane_type, p3.w);

   return format(R"V0G0N(
{{
   "name": {},
   "plane": [{}, 
             {}, 
             {}, 
             {}],
   "plane-type": {},
   "l-indices": [{}],
   "r-indices": [{}]
}}
)V0G0N",
                 json_encode(name),
                 str_precise(in_p3.x),
                 str_precise(in_p3.y),
                 str_precise(in_p3.z),
                 str_precise(in_p3.w),
                 plane_type,
                 int_arr_s(l_spixel_indices),
                 int_arr_s(r_spixel_indices));
}

// ---------------------------------------------------------------- Construction

This::This()
{
   p3s.resize(1);
   selected_index = 0;
   p3s[0].p3      = Plane(0.0, 0.0, 1.0, 0.0);
}

// ----------------------------------------------------------------------- Clear

void This::clear()
{
   p3s.clear();
   selected_index = -1;
}

// ------------------------------------------------------------------ operator==

bool This::operator==(const CalibPlaneSet& o) const noexcept
{
   return selected_index == o.selected_index and p3s == o.p3s;
}

bool This::operator!=(const CalibPlaneSet& o) const noexcept
{
   return !(*this == o);
}

// ----------------------------------------------------------- duplicates-report

string This::duplicates_report() const
{
   std::stringstream ss("");
   array<std::unordered_map<int, int>, 2> indices;

   auto process_image = [&](int ind) {
      auto& ref    = indices[size_t(ind)];
      auto image_l = (ind == 0) ? "L" : "R";

      for(auto i = 0u; i < p3s.size(); ++i) {
         const auto& cp = p3s[i];

         for(auto label : cp.indices(ind)) {
            auto ii = ref.find(label);
            if(ii == ref.end()) {
               ref[label] = int(i);
            } else {
               ss << format("   {} image, spixel {} appears in cp {} and {}\n",
                            image_l,
                            label,
                            ii->second,
                            i);
            }
         }
      }
   };

   for(auto ind = 0; ind < 2; ++ind) process_image(ind);

   return ss.str();
}

// ------------------------------------------------------------------- to-string

string This::to_string() const { return to_json_string(); }

// -------------------------------------------------------------- to-json-string

string This::to_json_string() const
{
   return format(
       R"V0G0N(
{{
   "p3s": [{}],
   "selected-index": {}
}}
)V0G0N",
       implode(begin(p3s),
               end(p3s),
               ", ",
               [&](const CalibPlane& cp) { return cp.to_json_string(); }),
       selected_index);
}

// ------------------------------------------------------------------- load/save

void load(CalibPlaneSet& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void save(const CalibPlaneSet& data, const string& fname) noexcept(false)
{
   std::string s;
   write(data, s);
   file_put_contents(fname, s);
}

void read(CalibPlaneSet& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);

   { // Sanity check...
      Json::Value node;
      CalibPlaneSet d0;
      write(data, node);
      read(d0, node);
      if(data != d0) FATAL("Error reading/writing CalibPlaneSet");
   }
}

void write(const CalibPlaneSet& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

void read(CalibPlaneSet& data, const Json::Value& root) noexcept(false)
{
   auto plane_type_from_p3 = [](const Plane& p3) {
      if(p3.x != 0.0) return 0;
      if(p3.y != 0.0) return 1;
      if(p3.z != 0.0) return 2;
      return 2;
   };

   data.clear();

   if(!has_key(root, "p3s"))
      throw std::runtime_error(
          "failed to find node 'p3s' parsing CalibPlaneSet");
   auto p3s = get_key(root, "p3s");
   if(!p3s.isArray())
      throw std::runtime_error(
          "expected an array for node 'p3s', parsing CalibPlaneSet");
   data.p3s.resize(p3s.size());
   for(auto i = 0u; i < p3s.size(); ++i) {
      auto src  = p3s[i];
      auto& dst = data.p3s[i];

      if(has_key(src, "name"))
         json_load(get_key(src, "name"), dst.name);
      else
         dst.name = ""s;

      if(has_key(src, "plane-type")) {
         json_load(get_key(src, "plane-type"), dst.plane_type);
         if(dst.plane_type < 0 || dst.plane_type >= 2)
            dst.plane_type = plane_type_from_p3(dst.p3);
         if(dst.plane_type == 0) dst.p3.xyz() = Vector3(1, 0, 0);
         if(dst.plane_type == 1) dst.p3.xyz() = Vector3(0, 1, 0);
         if(dst.plane_type == 2) dst.p3.xyz() = Vector3(0, 0, 1);
      } else {
         // Make sure 'p3' matches dst.plane_type
         dst.plane_type = plane_type_from_p3(dst.p3);
      }

      json_load(get_key(src, "plane"), dst.p3);
      json_load(get_key(src, "l-indices"), dst.l_spixel_indices);
      json_load(get_key(src, "r-indices"), dst.r_spixel_indices);
   }

   json_load(get_key(root, "selected-index"), data.selected_index);
}

void write(const CalibPlaneSet& data, Json::Value& node) noexcept(false)
{
   node = parse_json(data.to_json_string());
}

void load(vector<CalibPlaneSet>& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void save(const vector<CalibPlaneSet>& data,
          const string& fname) noexcept(false)
{
   std::string s;
   write(data, s);
   file_put_contents(fname, s);
}

void read(vector<CalibPlaneSet>& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);
}

void write(const vector<CalibPlaneSet>& data, std::string& out) noexcept(false)
{
   Json::Value node;
   write(data, node);
   std::stringstream ss("");
   ss << node;
   out = ss.str();
}

void read(vector<CalibPlaneSet>& data, const Json::Value& node) noexcept(false)
{
   if(!node.isArray()) throw std::runtime_error("expected an array");
   data.resize(node.size());
   for(auto i = 0u; i < data.size(); ++i) read(data[i], node[i]);
}

void write(const vector<CalibPlaneSet>& data, Json::Value& node) noexcept(false)
{
   Json::Value root(Json::arrayValue);
   root.resize(unsigned(data.size()));
   for(auto i = 0u; i < data.size(); ++i) write(data[i], root[i]);
   node = root;
}

} // namespace perceive::calibration
