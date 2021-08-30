
#pragma once

#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive::fowlkes
{
struct Params final : public MetaCompatible
{
   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   real min_track_seconds      = 2.0; // A track must have at least 2 seconds
   unsigned smooth_kz_filter_m = 9;   // 9, 4
   unsigned smooth_kz_filter_k = 4;
};

} // namespace perceive::fowlkes
