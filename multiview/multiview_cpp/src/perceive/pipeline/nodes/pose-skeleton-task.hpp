
#pragma once

#include "stdinc.hpp"

#include "convert_to_lab.hpp"
#include "input_images.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::pose_skeleton_init
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   pose_skeleton::Params op_params;

   bool feedback{false};
   std::string out_dir{"/tmp"};
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   Params p;
   shared_ptr<const copy_sensor_images::Result> copy_images_result;
   PoseSkeletonExec::Result op;

   const SceneDescription& scene_desc() const noexcept
   {
      return copy_images_result->scene_desc();
   }
};

class Task : public PipelineTask<Params, Result>
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   CUSTOM_NEW_DELETE(Task)

   Task();
   Task(const Task&) = delete;
   Task(Task&&)      = delete;
   ~Task();
   Task& operator=(const Task&) = delete;
   Task& operator=(Task&&) = delete;

 protected:
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;
};

}; // namespace perceive::pipeline::pose_skeleton_init

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::pose_skeleton_init::Params)
} // namespace perceive
