
#pragma once

#include "stdinc.hpp"

#include "convert_to_lab.hpp"
#include "cuda/perceive/graphics/slic.hpp"
#include "input_images.hpp"
#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::run_f2d
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned sensor_index{0};
   ImageFeatures2d::Params f2d;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   Params p;
   shared_ptr<const input_images::Result> image_result;
   shared_ptr<const convert_to_lab::Result> lab_conversion;
   ImageFeatures2d f2d;

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
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;

 private:
   cuda::slic::SLIC cuda_slic_context;
};

}; // namespace perceive::pipeline::run_f2d

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::run_f2d::Params)
} // namespace perceive
