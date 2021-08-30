
#pragma once

#include "cli-args.hpp"
#include "perceive/cost-functions/fowlkes/fowlkes-params.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive::pipeline
{
struct PipelineInput final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(PipelineInput)

   virtual ~PipelineInput() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   Json::Value task_params = Json::Value{Json::nullValue};
   // Json::Value frame_params    = Json::Value{Json::nullValue};
   perceive::fowlkes::Params fowlkes_params;
};

real lookup_person_radius(const Json::Value& frame_params);

} // namespace perceive::pipeline

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::PipelineInput)
} // namespace perceive
