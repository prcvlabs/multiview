
#pragma once

#include "stdinc.hpp"

#include "get_xy_mapping.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "run_f2d.hpp"

namespace perceive::pipeline::create_slic_lookup
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned sensor_index{0};
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const get_xy_mapping::Result> mapping_result;
   shared_ptr<const run_f2d::Result> f2d_result;

   IntImage slic_lookup;

   const SceneDescription& scene_desc() const
   {
      return mapping_result->scene_desc();
   }
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

}; // namespace perceive::pipeline::create_slic_lookup

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::create_slic_lookup::Params)
} // namespace perceive
