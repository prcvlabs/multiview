
#pragma once

#include "stdinc.hpp"

#include "load_scene_description.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::bcam_init
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned cam_num{0};
   unsigned width{800};  // format and
   unsigned height{600}; // intrinsic parameters
   real K_fx{200.0};
   real K_fy{200.0};
   real K_pptx{400.0};
   real K_ppty{300.0};
   bool use_calib_roi{true};

   Matrix3r K() const noexcept; // for rectified image
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const load_scene_description::Result> load_result;

   BinocularCamera bcam;
   unsigned width{0}; // Dimensions of working data
   unsigned height{0};
   const SceneDescription& scene_desc() const
   {
      return *load_result->scene_desc;
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

}; // namespace perceive::pipeline::bcam_init

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::bcam_init::Params)
} // namespace perceive
