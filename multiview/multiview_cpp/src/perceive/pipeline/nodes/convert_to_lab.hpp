
#pragma once

#include "cuda/perceive/graphics/rgb-to-lab.hpp"
#include "input_images.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "stdinc.hpp"

namespace perceive::pipeline::convert_to_lab
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

   shared_ptr<const input_images::Result> input_image_result;
   shared_ptr<const ARGBImage> argb_image;
   shared_ptr<const LABImage> lab_image;

   const SceneDescription& scene_desc() const
   {
      return input_image_result->scene_desc();
   }
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;

 private:
   cuda::ARGBToLABConversion lab_conversion;
};

}; // namespace perceive::pipeline::convert_to_lab

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::convert_to_lab::Params)
} // namespace perceive
