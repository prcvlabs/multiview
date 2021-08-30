
#pragma once

#include "stdinc.hpp"

#include "bcam_init.hpp"
#include "perceive/cost-functions/slic-3d/slic-3d.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "run_f2d.hpp"

namespace perceive::pipeline::slic_3d_init
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned cam_num{0};
   Slic3d::Params slic3d;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const bcam_init::Result> bcam_result;
   std::vector<shared_ptr<const run_f2d::Result>> f2d_result;
   std::vector<Slic3d::PlaneResult> plane_result;
   const SceneDescription& scene_desc() const
   {
      return bcam_result->scene_desc();
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

}; // namespace perceive::pipeline::slic_3d_init

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::slic_3d_init::Params)
} // namespace perceive
