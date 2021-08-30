
#include "pipeline-output.hpp"

#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/string-utils.hpp"

#define This PipelineOutput

namespace perceive
{
const vector<MemberMetaData>& This::meta_data() const noexcept
{
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(This, COMPATIBLE_OBJECT, config, true));
      m.push_back(MAKE_META(This, JSON_VALUE, params, true));
      m.push_back(MAKE_META(This, JSON_VALUE, track_results, true));
      return m;
   };

   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

// ----------------------------------------------------------- tracks-to-fowlkes
//
FowlkesResult This::tracks_to_fowlkes() const noexcept
{
   FowlkesResult fowlkes_ret;
   try {
      ::perceive::read(fowlkes_ret, track_results);
   } catch(std::exception& e) {
      FATAL(
          format("should /always/ be able to unpack track-results, aborting"));
   }
   return fowlkes_ret;
}

LocalizationData::Params This::localization_params() const noexcept(false)
{
   return read_localization_params(params);
}

} // namespace perceive
