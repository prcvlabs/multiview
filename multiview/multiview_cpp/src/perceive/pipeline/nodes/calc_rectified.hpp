
#pragma once

#include "stdinc.hpp"

#include "bcam_init.hpp"
#include "cuda/perceive/graphics/remap.hpp"
#include "get_xy_mapping.hpp"
#include "input_images.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::calc_rectified
{
struct Params final : public MetaCompatible
{
   enum InterpolationMethod : unsigned {
      NEAREST  = cv::INTER_NEAREST,
      LINEAR   = cv::INTER_LINEAR,
      CUBIC    = cv::INTER_CUBIC,
      LANCZOS4 = cv::INTER_LANCZOS4
   };

   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned sensor_index{0};
   bool use_cuda_if_available{false};
   InterpolationMethod interpolation_method{CUBIC};

   friend string str(Params::InterpolationMethod method) noexcept;
};

Params::InterpolationMethod to_interpolation_method(const string_view) noexcept;

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const input_images::Result> image_result;
   shared_ptr<const get_xy_mapping::Result> mapping_result;

   cv::Mat rectified;

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

 private:
   cuda::Remapper cuda_remapper;
};

}; // namespace perceive::pipeline::calc_rectified

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::calc_rectified::Params)
} // namespace perceive
