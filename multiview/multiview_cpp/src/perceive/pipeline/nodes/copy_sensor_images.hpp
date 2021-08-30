
#pragma once

#include "stdinc.hpp"

#include "perceive/scene/scene-description.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

#include "load_scene_description.hpp"

namespace perceive::pipeline::copy_sensor_images
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned frame_num{0};
   bool export_frames{false};
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const load_scene_description::Result> scene_result;
   SceneDescription::FrameImages images;

   const SceneDescription& scene_desc() const noexcept
   {
      return *scene_result->scene_desc;
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

}; // namespace perceive::pipeline::copy_sensor_images

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::copy_sensor_images::Params)
} // namespace perceive
