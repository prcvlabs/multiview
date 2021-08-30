
#include "track-point.hpp"

#define This TrackPoint

namespace perceive
{
string This::to_string() const noexcept
{
   return format("[{}, {}, [{}, {}], {}\u00B0, {}]",
                 x,
                 y,
                 t,
                 p3d_index,
                 to_degrees(angle_normalise2(gaze_direction)),
                 str(pose));
}

string This::to_json_string() const noexcept
{
   const auto s = format(
       "[{:7.4f}, {:7.4f}, {}, {}, {}, {}]",
       x,
       y,
       t,
       ((std::isfinite(gaze_direction)) ? str(gaze_direction) : "null"s),
       p3d_index,
       json_encode(format("{}", pose_to_char(pose))));
   return s;
}

Json::Value This::to_json() const noexcept
{
   Json::Value o{Json::arrayValue};
   o.resize(5);
   o[0] = real(x);
   o[1] = real(y);
   o[2] = t;
   if(std::isfinite(gaze_direction))
      o[3] = real(gaze_direction);
   else
      o[3] = Json::Value{Json::nullValue};
   o[4] = p3d_index;
   o[5] = format("{}", pose_to_char(pose));
   return o;
}

bool This::load_json(const Json::Value& x) noexcept(false)
{
   if(x.isArray() and (x.size() >= 4) and (x.size() <= 6)) {
      read_with_defaults(x);
      return true;
   }

   return false;
}

void This::read_with_defaults(const Json::Value& o,
                              const TrackPoint* defaults) noexcept
{
   TrackPoint x = (defaults) ? *defaults : *this;

   if(o.isArray() and (o.size() >= 4) and (o.size() <= 6)) {
      json_load(o[0], x.x);
      json_load(o[1], x.y);
      json_load(o[2], x.t);
      if(o[3].isNumeric())
         json_load(o[3], x.gaze_direction);
      else
         x.gaze_direction = std::numeric_limits<float>::quiet_NaN();
      if(o.size() >= 5) json_load(o[4], x.p3d_index);
      if(o.size() >= 6) {
         string s;
         json_load(o[5], s);
         if(s.size() == 1) x.pose = to_pose_annotation(s[0]);
      }
      *this = x;
   } else {
      WARN(format(
          "read TrackPoint: expected Json array of size 4. No data was read"));
   }
}

} // namespace perceive
