
#include "pose-annotation.hpp"

#define This PoseAnnotation

namespace perceive
{
const char* str(const PoseAnnotation& o) noexcept
{
   switch(o) {
#define CASE(x) \
   case PoseAnnotation::x: return #x
      CASE(NONE);
      CASE(STAND);
      CASE(WALK);
      CASE(SIT);
      CASE(LAY);
      CASE(PHONE);
      CASE(OTHER);
#undef CASE
   }
   return "NONE";
}

PoseAnnotation to_pose_annotation(const string_view s) noexcept
{
#define CASE(x) \
   if(s == #x) return PoseAnnotation::x;
   CASE(NONE);
   CASE(STAND);
   CASE(WALK);
   CASE(SIT);
   CASE(LAY);
   CASE(PHONE);
   CASE(OTHER);
#undef CASE
   return PoseAnnotation::NONE;
}

PoseAnnotation to_pose_annotation(const int val) noexcept
{
#define CASE(x) \
   if(val == int(PoseAnnotation::x)) return PoseAnnotation::x;
   CASE(NONE);
   CASE(STAND);
   CASE(WALK);
   CASE(SIT);
   CASE(LAY);
   CASE(PHONE);
   CASE(OTHER);
#undef CASE
   return PoseAnnotation::NONE;
}

PoseAnnotation to_pose_annotation(const char o) noexcept
{
   switch(o) {
   case '.': return PoseAnnotation::NONE;
   case 'S': return PoseAnnotation::STAND;
   case 'W': return PoseAnnotation::WALK;
   case 'I': return PoseAnnotation::SIT;
   case 'L': return PoseAnnotation::LAY;
   case 'P': return PoseAnnotation::PHONE;
   case 'O': return PoseAnnotation::OTHER;
   }
   return PoseAnnotation::NONE;
}

char pose_to_char(const PoseAnnotation& o) noexcept
{
   switch(o) {
   case PoseAnnotation::NONE: return '.';
   case PoseAnnotation::STAND: return 'S';
   case PoseAnnotation::WALK: return 'W';
   case PoseAnnotation::SIT: return 'I';
   case PoseAnnotation::LAY: return 'L';
   case PoseAnnotation::PHONE: return 'P';
   case PoseAnnotation::OTHER: return 'O';
   }
   return 'N';
}

} // namespace perceive
