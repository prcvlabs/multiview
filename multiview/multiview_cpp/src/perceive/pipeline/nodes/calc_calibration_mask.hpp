
#pragma once

#include "stdinc.hpp"

#include "input_images.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::calc_calibration_mask
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

   shared_ptr<const input_images::Result> image_result;

   BinaryImage calib_mask;

   const SceneDescription& scene_desc() const
   {
      return image_result->scene_desc();
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

}; // namespace perceive::pipeline::calc_calibration_mask

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::calc_calibration_mask::Params)
} // namespace perceive
