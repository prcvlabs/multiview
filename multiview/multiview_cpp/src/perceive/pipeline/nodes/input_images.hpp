
#pragma once

#include "stdinc.hpp"

#include "copy_sensor_images.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include <opencv2/opencv.hpp>

namespace perceive::pipeline::input_images
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned sensor_index{0};
   bool raw_image_equalize{false};
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const copy_sensor_images::Result> copy_image_result;

   cv::Mat raw;
   cv::Mat equal;
   cv::Mat grey;

   const SceneDescription& scene_desc() const noexcept
   {
      return copy_image_result->scene_desc();
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

}; // namespace perceive::pipeline::input_images

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::input_images::Params)
} // namespace perceive
