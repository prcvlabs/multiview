
#include "fowlkes-params.hpp"

#define This Params

namespace perceive::fowlkes
{
const vector<MemberMetaData>& This::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(This, REAL, min_track_seconds, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

} // namespace perceive::fowlkes
