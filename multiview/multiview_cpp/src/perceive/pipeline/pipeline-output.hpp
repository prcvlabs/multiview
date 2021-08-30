
#pragma once

#include "cli-args.hpp"
#include "load-params.hpp"
#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
struct PipelineOutput final : public MetaCompatible
{
 public:
   CUSTOM_NEW_DELETE(PipelineOutput)

   virtual ~PipelineOutput() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   pipeline::CliArgs config;
   Json::Value params;
   Json::Value track_results;

   FowlkesResult tracks_to_fowlkes() const noexcept;

   LocalizationData::Params localization_params() const noexcept(false);
};

META_READ_WRITE_LOAD_SAVE(PipelineOutput)

} // namespace perceive
