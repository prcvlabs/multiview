
#pragma once

#include "stdinc.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive::pipeline::load_scene_description
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool operator==(const Params& o) const noexcept;
   bool operator!=(const Params& o) const noexcept { return !(*this == o); }

   bool feedback       = false;
   std::string out_dir = "/tmp"s;
   shared_ptr<const SceneDescription> scene_desc;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   // This should always be non-null. If the scene-description
   // cannot be set, then the result object should not be created.
   shared_ptr<const SceneDescription> scene_desc;
   real frame_duration = dNAN;
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

// template<typename RunData>
// shared_ptr<const SceneDescription>
// ggg_scene_description(const RunData& data)
// {
//    auto ret = data.match_result<Result>("load_scene_description");
//    return ret ? ret->scene_description : nullptr;
// }

}; // namespace perceive::pipeline::load_scene_description

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::load_scene_description::Params)
} // namespace perceive
