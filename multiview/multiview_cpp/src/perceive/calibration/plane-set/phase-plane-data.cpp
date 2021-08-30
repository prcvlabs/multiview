
#include "stdinc.hpp"

#include "phase-plane-data.hpp"

#include "perceive/scene/scene-description-info.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/container.hpp"
#include "perceive/utils/file-system.hpp"

#define This PhasePlaneData

static std::atomic<int> plane_info_ids{0};

namespace perceive::calibration
{
// ------------------------------------------------------------------ operator==

bool This::operator==(const PhasePlaneData& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(scene_key) && TEST(p3s) && TEST(s_infos) && TEST(rp_infos)
          && TEST(highlighted_p3);
#undef TEST
}

bool This::operator!=(const PhasePlaneData& o) const noexcept
{
   return !(*this == o);
}

bool This::PlaneInfo::operator==(const PlaneInfo& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(name) && TEST(type) && TEST(d) && TEST(do_optimize)
          && TEST(selected);
#undef TEST
}

bool This::PlaneInfo::operator!=(const PlaneInfo& o) const noexcept
{
   return !(*this == o);
}

bool This::RayPlanePlaneInfo::operator==(const RayPlanePlaneInfo& o) const
    noexcept
{
#define TEST(x) (x == o.x)
   return TEST(sensor_key) && TEST(inds[0]) && TEST(inds[1]) && TEST(x);
#undef TEST
}

bool This::RayPlanePlaneInfo::operator!=(const RayPlanePlaneInfo& o) const
    noexcept
{
   return !(*this == o);
}

bool This::StillInfo::operator==(const StillInfo& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(camera_key) && TEST(slic_size) && TEST(slic_compactness)
          && TEST(do_optimize);
#undef TEST
}

bool This::StillInfo::operator!=(const StillInfo& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------------------------------ init

bool This::init(const SceneDescriptionInfo& scene_desc) noexcept
{
   scene_key = scene_desc.scene_key;
   p3s.resize(scene_desc.known_planes.size());
   std::transform(cbegin(scene_desc.known_planes),
                  cend(scene_desc.known_planes),
                  begin(p3s),
                  [&](const auto& p) {
                     const auto& p3 = p.second;
                     PlaneInfo o;
                     // o.id          = ++plane_info_ids;
                     o.name        = p.first;
                     o.type        = (p3.x != 0.0) ? 0 : (p3.y != 0.0) ? 1 : 2;
                     o.d           = p3.d();
                     o.do_optimize = false;
                     return o;
                  });

   s_infos.resize(scene_desc.bcam_keys.size());
   std::transform(cbegin(scene_desc.bcam_keys),
                  cend(scene_desc.bcam_keys),
                  begin(s_infos),
                  [&](const auto& key) {
                     StillInfo s;
                     s.camera_key = key;
                     return s;
                  });
   return true;
}

string This::to_string() const noexcept { return to_json_string(); }

string This::to_json_string() const noexcept
{
   auto encode_p3 = [&](const PlaneInfo& p) -> string {
      return format(R"V0G0N({{
         "name": {},
         "type": {},
         "d": {},
         "do-optimize": {},
         "selected": [{}]
      {}}})V0G0N",
                    json_encode(p.name),
                    ((p.type >= 0 and p.type <= 2) ? p.type : 2),
                    p.d,
                    str(p.do_optimize),
                    implode(cbegin(p.selected),
                            cend(p.selected),
                            ",\n\n                      ",
                            [&](const auto& x) -> string {
                               return format("[{}]",
                                             implode(cbegin(x), cend(x), ", "));
                            }),
                    "");
   };

   auto encode_s_info = [&](const StillInfo& s) -> string {
      return format(R"V0G0N({{
         "camera-key": {},
         "slic-size": {},
         "slic-compactness": {},
         "do-optimize": {}
      {}}})V0G0N",
                    json_encode(s.camera_key),
                    s.slic_size,
                    s.slic_compactness,
                    str(s.do_optimize),
                    "");
   };

   auto encode_rp_info = [&](const RayPlanePlaneInfo& rp) -> string {
      if(rp.is_line())
         return format("[{}, {}, {}, {}, {}]",
                       json_encode(rp.sensor_key),
                       rp.inds[0],
                       rp.inds[1],
                       rp.x.x,
                       rp.x.y);
      return format("[{}, {}, {}, {}, {}, {}]",
                    json_encode(rp.sensor_key),
                    rp.inds[0],
                    rp.inds[1],
                    rp.inds[2],
                    rp.x.x,
                    rp.x.y);
   };

   return format(
       R"V0G0N({{
   "scene-key": {},

   "highlighted-p3": {},

   "p3s": [{}],

   "s_infos": [{}],

   "rp_infos": [{}]
}}{})V0G0N",
       json_encode(scene_key),
       highlighted_p3,
       implode(cbegin(p3s), cend(p3s), ",\n      ", encode_p3),
       implode(cbegin(s_infos), cend(s_infos), ",\n      ", encode_s_info),
       implode(cbegin(rp_infos), cend(rp_infos), ",\n      ", encode_rp_info),
       "");
}

// ------------------------------------------------------------------------ Load

void load(PhasePlaneData& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

// ------------------------------------------------------------------------ Save

void save(const PhasePlaneData& data, const string& fname) noexcept(false)
{
   std::string s;
   write(data, s);
   file_put_contents(fname, s);
}

// ------------------------------------------------------------------------ Read

void read(PhasePlaneData& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);
}

// ----------------------------------------------------------------------- Write

void write(const PhasePlaneData& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

// ------------------------------------------------------------------------ Read

void read(PhasePlaneData& data, const Json::Value& node) noexcept(false)
{
   std::string err = "";

   auto from_json = [&](const Json::Value& root) {
      json_load(get_key(root, "scene-key"), data.scene_key);
      json_load(get_key(root, "highlighted-p3"), data.highlighted_p3);

      try {
         data.p3s.clear();
         auto arr = get_key(root, "p3s");
         if(!arr.isArray())
            throw std::runtime_error("expected array of values");
         data.p3s.resize(arr.size());
         std::vector<int> all_keys;
         for(auto i = 0u; i < arr.size(); ++i) {
            const auto& node = arr[i];
            auto& p          = data.p3s[i];
            json_load(get_key(node, "name"), p.name);
            json_load(get_key(node, "type"), p.type);
            json_load(get_key(node, "d"), p.d);
            json_load(get_key(node, "do-optimize"), p.do_optimize);

            { // get selected
               auto sarr = get_key(node, "selected");
               if(!sarr.isArray())
                  throw std::runtime_error(
                      "selected key must be array of values");

               p.selected.resize(sarr.size());
               vector<int> selected;
               for(auto i = 0u; i < p.selected.size(); ++i) {
                  selected.clear();
                  json_load(sarr[i], selected);
                  p.selected[i].clear();
                  p.selected[i].insert(cbegin(selected), cend(selected));
               }
            }

            if(p.type < 0 or p.type > 2) p.type = 2;
            if(!std::isfinite(p.d)) p.d = 0.0;
         }

      } catch(std::runtime_error& e) {
         LOG_ERR(
             format("error reading phase-plane-data p3-info: {}", e.what()));
         data.p3s.clear();
      }

      try {
         data.s_infos.clear();
         auto arr = get_key(root, "s_infos");
         if(!arr.isArray())
            throw std::runtime_error("expected array of values");
         data.s_infos.resize(arr.size());
         for(auto i = 0u; i < arr.size(); ++i) {
            const auto& node = arr[i];
            auto& s          = data.s_infos[i];
            json_load(get_key(node, "camera-key"), s.camera_key);
            json_load(get_key(node, "slic-size"), s.slic_size);
            json_load(get_key(node, "slic-compactness"), s.slic_compactness);
            json_load(get_key(node, "do-optimize"), s.do_optimize);
         }
      } catch(std::runtime_error& e) {
         LOG_ERR(
             format("error reading phase-plane-data s-info: {}", e.what()));
         data.s_infos.clear();
      }

      try {
         data.rp_infos.clear();
         auto arr = get_key(root, "rp_infos");
         if(arr.isArray()) {
            data.rp_infos.resize(arr.size());
            for(auto i = 0u; i < data.rp_infos.size(); ++i) {
               const auto& o = arr[i];
               if(!o.isArray() or !(o.size() == 5 or o.size() == 6))
                  throw std::runtime_error(
                      "rp info must be an array of size 5 or 6");
               json_load(o[0], data.rp_infos[i].sensor_key);
               json_load(o[1], data.rp_infos[i].inds[0]);
               json_load(o[2], data.rp_infos[i].inds[1]);
               if(o.size() == 5) {
                  json_load(o[3], data.rp_infos[i].x.x);
                  json_load(o[4], data.rp_infos[i].x.y);
               } else {
                  json_load(o[3], data.rp_infos[i].inds[2]);
                  json_load(o[4], data.rp_infos[i].x.x);
                  json_load(o[5], data.rp_infos[i].x.y);
               }
            }
         }
      } catch(std::runtime_error& e) {
         // "rp_infos": [{}],
         LOG_ERR(format("error reading rp-info: {}", e.what()));
         data.rp_infos.clear();
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

// ----------------------------------------------------------------------- Write

void write(const PhasePlaneData& data, Json::Value& node) noexcept(false)
{
   node = parse_json(data.to_json_string());
}

} // namespace perceive::calibration
