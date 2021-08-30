
#pragma once

#include "stdinc.hpp"

#include "bcam_init.hpp"
#include "load_scene_description.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include <opencv2/opencv.hpp>

namespace perceive::pipeline::get_xy_mapping
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

   shared_ptr<const bcam_init::Result> bcam_result;
   shared_ptr<const load_scene_description::Result> scene_result;

   cv::Mat mapx, mapy; // rectification maps for sensor

#ifdef WITH_CUDA
   cv::cuda::GpuMat mapx_gpu, mapy_gpu; // rectification maps in GPU memory
#endif

   const SceneDescription& scene_desc() const
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

}; // namespace perceive::pipeline::get_xy_mapping

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::get_xy_mapping::Params)
} // namespace perceive
