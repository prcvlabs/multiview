
#include "track.hpp"

#include "json/json.h"

#define This Track

namespace perceive
{
string str(Track::Label o)
{
   switch(o) {
   case Track::UNLABELLED: return "UNLABELLED"s;
   case Track::CUSTOMER: return "CUSTOMER"s;
   case Track::EMPLOYEE: return "EMPLOYEE"s;
   }
   FATAL("logic error");
   return "LOGIC ERROR";
}

This::Label to_track_label(const string_view s) noexcept(false)
{
#define CASE(x) \
   if(s == #x) return Track::x
   CASE(UNLABELLED);
   CASE(CUSTOMER);
   CASE(EMPLOYEE);
#undef CASE
   throw std::runtime_error(format("unknown track type: '{}'", s));
}

// ------------------------------------------------------------------ operator==

bool This::operator==(const Track& o) const noexcept
{
#define TEST(x) (x == o.x)
#define TEST_NUM(x) (float_is_same(x, o.x))
   return TEST(label) and TEST(id) and TEST(path) and TEST_NUM(cost)
          and TEST_NUM(height);
#undef TEST
#undef TEST_NUM
}
bool This::operator!=(const Track& o) const noexcept { return !(*this == o); }

// ---------------------------------------------------------------- memory-usage
size_t This::memory_usage() const noexcept
{
   Expects(path.capacity() >= path.size());
   return sizeof(This) + (path.capacity() - path.size()) * sizeof(TrackPoint)
          + std::accumulate(cbegin(path),
                            cend(path),
                            size_t(0),
                            [](size_t sz, const TrackPoint& tp) {
                               return tp.memory_usage();
                            });
}

// -------------------------------------------------------------------- is-valid

bool This::is_valid() const noexcept
{
   if(id < 0) return false;
   for(auto i = 1u; i < path.size(); ++i)
      if(!(path[i - 1].t + 1 == path[i].t)) return false;
   return true;
}

Line1Di This::path_time_range() const noexcept
{
   return path.size() == 0 ? Line1Di{0, 0}
                           : Line1Di{path.front().t, path.back().t + 1};
}

bool This::before(const Track& tt) const noexcept
{
   if(empty() or tt.empty()) return false;
   return path_time_range().b <= tt.path_time_range().a;
}

bool This::after(const Track& tt) const noexcept
{
   if(empty() or tt.empty()) return false;
   return path_time_range().a >= tt.path_time_range().b;
}

// ----------------------------------------
std::string This::brief_info() const noexcept
{
   return format("Track: id={}, label={}, height={}, cost={}, size={}",
                 id,
                 str(label),
                 height,
                 cost,
                 path.size());
}

std::string This::to_string() const noexcept { return to_json_string(); }

static Json::Value fp_info_to_json(const TrackFalsePositiveInfo& fp_info)
{
   Json::Value x{Json::objectValue};
   x["prob_fp_p"]        = json_save(fp_info.prob_fp_p);
   x["dist_p"]           = json_save(fp_info.dist_p);
   x["lab_p"]            = json_save(fp_info.lab_p);
   x["duration_p"]       = json_save(fp_info.duration_p);
   x["score"]            = json_save(fp_info.score);
   x["is_true_positive"] = json_save(fp_info.is_true_positive);
   return x;
}

std::string This::to_json_string() const noexcept
{
   const auto path_s
       = implode(cbegin(path), cend(path), ", ", [&](const auto& x) {
            return x.to_json_string();
         });

   return format(R"V0G0N(
{{
   "id": {},
   "height": {},
   "cost": {},
   "label": {},
   "fp_info": {},
   "path": [{}]
}}
{})V0G0N",
                 id,
                 (std::isfinite(height) ? str(height) : "null"s),
                 (std::isfinite(cost) ? str(cost) : "null"s),
                 json_encode(str(label)),
                 trim_copy(indent(str(fp_info_to_json(fp_info)), 6)),
                 path_s,
                 "");
}

Json::Value This::to_json() const noexcept
{
   Json::Value o{Json::objectValue};
   o["id"]     = id;
   o["height"] = json_save(height);
   o["cost"]   = json_save(cost);
   o["label"]  = str(label);
   o["path"] = json_save_t(cbegin(path), cend(path), [&](const TrackPoint& x) {
      return x.to_json();
   });
   o["fp_info"] = fp_info_to_json(fp_info);
   return o;
}

void This::read_with_defaults(const Json::Value& o,
                              const Track* defaults) noexcept
{
   Track x;
   if(defaults != nullptr) x = *defaults;

   json_try_load_t<TrackPoint>(
       x.path,
       o,
       "path"s,
       [&](const Json::Value& node, TrackPoint& tp) { read(tp, node); },
       "reading Track"s,
       false);
   json_try_load_key(x.id, o, "id", "reading Track", false);
   json_try_load_key(x.height, o, "height", "reading Track", false);

   if(!has_key(o, "height")) { x.height = NAN; }

   json_try_load_key(x.cost, o, "cost", "reading Track", false);

   if(has_key(o, "label")) {
      try {
         const string val = o["label"].asString();
         x.label          = to_track_label(val);
      } catch(std::runtime_error& e) {
         WARN(format("Track had spurious label value"));
      }
   }

   *this = std::move(x);
}

string quick_view(const vector<Track>& tts) noexcept
{
   auto quick_view_track = [&](const Track& tt) {
      return format(
          "{{ id = {}, [{}] }}",
          tt.id,
          implode(
              cbegin(tt.path), cend(tt.path), ", ", [&](const TrackPoint& tp) {
                 return format("({}, {}, {})", tp.x, tp.y, tp.t);
              }));
   };

   return implode(cbegin(tts), cend(tts), ",\n", quick_view_track);
}

} // namespace perceive
